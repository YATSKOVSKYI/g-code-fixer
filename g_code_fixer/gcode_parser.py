from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class PositionState:
    """Tracks the latest machine position used while parsing."""

    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    e: Optional[float] = None
    f: Optional[float] = None

    def copy(self) -> "PositionState":
        return PositionState(self.x, self.y, self.z, self.e, self.f)


@dataclass
class GCodeCommand:
    """Represents a single line of G-code."""

    original: str
    command: Optional[str] = None
    params: Dict[str, float] = field(default_factory=dict)
    comment: Optional[str] = None
    index: int = 0
    layer: Optional[int] = None
    state_before: PositionState = field(default_factory=PositionState)
    state_after: PositionState = field(default_factory=PositionState)

    def is_move(self) -> bool:
        return self.command in {"G0", "G00", "G1", "G01"}

    def format(self) -> str:
        """Return the command formatted back to G-code."""
        if self.command is None:
            return self.original

        tokens = [self.command]
        ordered_keys = ("X", "Y", "Z", "E", "F")
        used = set()
        for key in ordered_keys:
            if key in self.params:
                value = self.params[key]
                tokens.append(f"{key}{value:.5f}".rstrip("0").rstrip("."))
                used.add(key)
        for key in sorted(k for k in self.params.keys() if k not in used):
            value = self.params[key]
            tokens.append(f"{key}{value:.5f}".rstrip("0").rstrip("."))

        remainder = " ".join(tokens)
        if self.comment:
            return f"{remainder} ;{self.comment}"
        return remainder


@dataclass
class GCodeSegment:
    """Represents a single printable or travel segment inside one layer."""

    start: Tuple[float, float]
    end: Tuple[float, float]
    layer: int
    is_extrusion: bool
    end_command_index: int


@dataclass
class GCodeNode:
    """Represents a draggable point at the end of a move command."""

    command_index: int
    position: Tuple[float, float]
    layer: int
    is_extrusion: bool


class GCodeModel:
    """Parses G-code into commands and provides helpers for editing."""

    layer_height_tolerance: float = 0.0009

    def __init__(self, lines: List[str]):
        self.commands: List[GCodeCommand] = []
        self.layers: List[float] = []
        self.layer_map: Dict[float, int] = {}
        self._parse(lines)

    def _parse(self, lines: List[str]) -> None:
        state = PositionState()
        current_layer = -1
        current_layer_z = None
        for idx, raw_line in enumerate(lines):
            stripped = raw_line.strip()
            if not stripped:
                command = GCodeCommand(original=raw_line.rstrip("\n"), comment=None, index=idx)
                self.commands.append(command)
                continue

            if ";" in stripped:
                code_part, comment_part = stripped.split(";", 1)
                comment_part = comment_part.strip()
                stripped = code_part.strip()
            else:
                comment_part = None

            if not stripped:
                # Line with only comment
                command = GCodeCommand(
                    original=raw_line.rstrip("\n"),
                    comment=comment_part,
                    index=idx,
                )
                self.commands.append(command)
                continue

            tokens = stripped.split()
            cmd = tokens[0].upper()
            params: Dict[str, float] = {}
            for token in tokens[1:]:
                key = token[0].upper()
                try:
                    params[key] = float(token[1:])
                except ValueError:
                    # Preserve token in comment if malformed.
                    if comment_part:
                        comment_part += f" {token}"
                    else:
                        comment_part = token

            command = GCodeCommand(
                original=raw_line.rstrip("\n"),
                command=cmd,
                params=params,
                comment=comment_part,
                index=idx,
            )
            # Determine layer based on Z change
            if command.is_move():
                z_value = params.get("Z", state.z)

                if z_value is not None:
                    if current_layer_z is None or abs(z_value - current_layer_z) > self.layer_height_tolerance:
                        current_layer_z = z_value
                        current_layer += 1
                        if z_value not in self.layer_map:
                            self.layer_map[z_value] = current_layer
                            self.layers.append(z_value)

                    command.layer = self.layer_map.get(current_layer_z, current_layer)
                else:
                    command.layer = current_layer
            else:
                command.layer = current_layer

            self.commands.append(command)

            for key, value in params.items():
                attr = key.lower()
                if hasattr(state, attr):
                    setattr(state, attr, value)

        self.recompute_states()

    def rebuild(self) -> List[str]:
        """Return the G-code as list of lines based on current commands."""
        output = []
        for command in self.commands:
            output.append(command.format())
        return output

    def iter_segments(self, layer: int) -> List[GCodeSegment]:
        """Return all line segments for a specific layer."""
        segments: List[GCodeSegment] = []
        for idx, command in enumerate(self.commands):
            if not command.is_move():
                continue
            if command.layer != layer:
                continue

            start_state = command.state_before
            end_state = command.state_after
            if start_state.x is None or start_state.y is None:
                continue
            if end_state.x is None or end_state.y is None:
                continue
            start = (start_state.x, start_state.y)
            end = (end_state.x, end_state.y)
            if start == end:
                continue

            extrusion = False
            if start_state.e is not None and end_state.e is not None:
                extrusion = (end_state.e - start_state.e) > 0.0

            segments.append(
                GCodeSegment(
                    start=start,
                    end=end,
                    layer=layer,
                    is_extrusion=extrusion,
                    end_command_index=idx,
                )
            )
        return segments

    def iter_nodes(self, layer: int) -> List[GCodeNode]:
        """Return draggable nodes for a specific layer."""
        nodes: List[GCodeNode] = []
        for idx, command in enumerate(self.commands):
            if not command.is_move():
                continue
            if command.layer != layer:
                continue
            end_state = command.state_after
            if end_state.x is None or end_state.y is None:
                continue
            start_state = command.state_before
            extrusion = False
            if start_state.e is not None and end_state.e is not None:
                extrusion = (end_state.e - start_state.e) > 0.0
            nodes.append(
                GCodeNode(
                    command_index=idx,
                    position=(end_state.x, end_state.y),
                    layer=layer,
                    is_extrusion=extrusion,
                )
            )
        return nodes

    def get_bounds(self, layer: Optional[int] = None) -> Optional[Tuple[float, float, float, float]]:
        """Return axis-aligned bounds (min_x, max_x, min_y, max_y) for commands."""
        xs: List[float] = []
        ys: List[float] = []
        for command in self.commands:
            if not command.is_move():
                continue
            if layer is not None and command.layer != layer:
                continue
            start = command.state_before
            end = command.state_after
            for point in (start, end):
                if point.x is not None and point.y is not None:
                    xs.append(point.x)
                    ys.append(point.y)
        if not xs or not ys:
            return None
        return min(xs), max(xs), min(ys), max(ys)

    def get_layer_count(self) -> int:
        return max((cmd.layer for cmd in self.commands if cmd.layer is not None), default=-1) + 1

    def update_command_position(self, command_index: int, x: Optional[float], y: Optional[float]) -> None:
        """Update the XY values of a command and recompute dependent states."""
        command = self.commands[command_index]
        if x is not None:
            command.params["X"] = x
        if y is not None:
            command.params["Y"] = y
        self.recompute_states()

    def recompute_states(self) -> None:
        """Recalculate state_before/state_after for all commands."""
        state = PositionState()
        for command in self.commands:
            command.state_before = state.copy()
            if command.command is None:
                command.state_after = state.copy()
                continue

            # Commands without explicit X/Y/Z/E params inherit previous state.
            for key, value in command.params.items():
                attr = key.lower()
                if hasattr(state, attr):
                    setattr(state, attr, value)
            command.state_after = state.copy()
