from __future__ import annotations

import math
import os
import time
from bisect import bisect_left
from pathlib import Path
from typing import Dict, List, Optional

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFontDatabase, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import (
    QApplication,
    QFileDialog,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsPathItem,
    QGraphicsLineItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSlider,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .gcode_parser import GCodeModel, GCodeNode, GCodeSegment


class StaticAnchorItem(QGraphicsEllipseItem):
    """Visual marker for a fixed (non-draggable) point."""

    def __init__(self, position: QPointF, radius: float = 0.4):
        super().__init__(-radius, -radius, radius * 2.0, radius * 2.0)
        self.setPos(position)
        self.setPen(QPen(QColor(180, 180, 180), 0))
        self.setBrush(QColor(200, 200, 200, 120))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setZValue(5)


class SegmentItem(QGraphicsLineItem):
    """Line segment connecting two move commands."""

    def __init__(self, segment: GCodeSegment, color: QColor):
        super().__init__()
        self.segment = segment
        self.start_handle: Optional["DraggableHandle"] = None
        self.end_handle: Optional["DraggableHandle"] = None
        self.static_start: Optional[QPointF] = None
        self.static_end: Optional[QPointF] = None
        self.setPen(QPen(color, 0))
        self.setZValue(0)
        self.setAcceptHoverEvents(True)
        self._base_color = color
        self._active_handle: Optional["DraggableHandle"] = None
        self._hovered = False

    def set_start_handle(self, handle: "DraggableHandle") -> None:
        self.start_handle = handle
        handle.attach_segment(self, is_start=True)
        self.static_start = None
        self.refresh()

    def set_end_handle(self, handle: "DraggableHandle") -> None:
        self.end_handle = handle
        handle.attach_segment(self, is_start=False)
        self.static_end = None
        self.refresh()

    def set_static_start(self, point: QPointF) -> None:
        self.static_start = point
        self.refresh()

    def set_static_end(self, point: QPointF) -> None:
        self.static_end = point
        self.refresh()

    def refresh(self) -> None:
        start = self.start_handle.scenePos() if self.start_handle else self.static_start
        end = self.end_handle.scenePos() if self.end_handle else self.static_end
        if start and end:
            self.setLine(start.x(), start.y(), end.x(), end.y())
        self._update_pen()

    def _update_pen(self) -> None:
        width = 0.0 if not self._hovered else 0.6
        color = QColor(self._base_color)
        if self._active_handle:
            color = QColor(255, 220, 0) if self.segment.is_extrusion else QColor(180, 220, 255)
        self.setPen(QPen(color, width))

    def hoverEnterEvent(self, event) -> None:
        self._hovered = True
        self._update_pen()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event) -> None:
        self._hovered = False
        self._update_pen()
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            scene_pos = event.scenePos()
            threshold = 3.0
            start_point = QPointF(self.line().x1(), self.line().y1())
            end_point = QPointF(self.line().x2(), self.line().y2())
            start_dist = math.hypot(scene_pos.x() - start_point.x(), scene_pos.y() - start_point.y())
            end_dist = math.hypot(scene_pos.x() - end_point.x(), scene_pos.y() - end_point.y())

            chosen: Optional["DraggableHandle"] = None
            if self.start_handle and start_dist <= threshold:
                chosen = self.start_handle
            if self.end_handle and end_dist <= threshold and (chosen is None or end_dist < start_dist):
                chosen = self.end_handle

            if chosen:
                self._active_handle = chosen
                self._hovered = True
                self._update_pen()
                chosen.setSelected(True)
                chosen.setPos(scene_pos)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._active_handle:
            self._active_handle.setPos(event.scenePos())
            for segment, _ in list(self._active_handle._segments):
                segment.refresh()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self._active_handle:
            handle = self._active_handle
            self._active_handle = None
            handle.setPos(event.scenePos())
            for segment, _ in list(handle._segments):
                segment.refresh()
            if handle.commit_callback:
                handle.commit_callback(handle, handle.scenePos())
            event.accept()
            return
        super().mouseReleaseEvent(event)


class DraggableHandle(QGraphicsEllipseItem):
    """Movable handle bound to the end of a move command."""

    def __init__(
        self,
        node: GCodeNode,
        color: QColor,
        commit_callback,
        radius: float = 0.8,
    ):
        super().__init__(-radius, -radius, radius * 2.0, radius * 2.0)
        self.node = node
        self.commit_callback = commit_callback
        self._segments: list[tuple[SegmentItem, bool]] = []
        self.setBrush(color)
        self.setPen(QPen(QColor(30, 30, 30), 0))
        self.setFlags(
            QGraphicsItem.GraphicsItemFlag.ItemIsMovable
            | QGraphicsItem.GraphicsItemFlag.ItemSendsScenePositionChanges
            | QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
        )
        self.setZValue(10)
        self.setPos(QPointF(*node.position))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)

    def attach_segment(self, segment: SegmentItem, is_start: bool) -> None:
        self._segments.append((segment, is_start))

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for segment, _ in self._segments:
                segment.refresh()
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        if self.commit_callback:
            position = self.scenePos()
            self.commit_callback(self, position)


class LayerSelector(QWidget):
    """Slider + spinbox combo to choose active layer."""

    layerChanged = pyqtSignal(int)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self._label = QLabel("Слой:", self)
        self._slider = QSlider(Qt.Orientation.Horizontal, self)
        self._slider.setMinimum(0)
        self._slider.setMaximum(0)
        self._slider.setEnabled(False)

        self._spin = QSpinBox(self)
        self._spin.setMinimum(0)
        self._spin.setMaximum(0)
        self._spin.setEnabled(False)

        layout.addWidget(self._label)
        layout.addWidget(self._slider, 1)
        layout.addWidget(self._spin)

        self._slider.valueChanged.connect(self._on_slider_changed)
        self._spin.valueChanged.connect(self._on_spin_changed)
        self._updates_blocked = False

    def set_layer_count(self, count: int) -> None:
        self._updates_blocked = True
        if count <= 0:
            self._slider.setEnabled(False)
            self._spin.setEnabled(False)
            self._slider.setMaximum(0)
            self._spin.setMaximum(0)
            self._slider.setValue(0)
            self._spin.setValue(0)
        else:
            self._slider.setEnabled(True)
            self._spin.setEnabled(True)
            self._slider.setMaximum(count - 1)
            self._spin.setMaximum(count - 1)
            self._slider.setValue(0)
            self._spin.setValue(0)
        self._updates_blocked = False
        if count > 0:
            self.layerChanged.emit(0)

    def set_layer(self, layer: int) -> None:
        self._updates_blocked = True
        self._slider.setValue(layer)
        self._spin.setValue(layer)
        self._updates_blocked = False

    def _on_slider_changed(self, value: int) -> None:
        if self._updates_blocked:
            return
        self._spin.setValue(value)
        self.layerChanged.emit(value)

    def _on_spin_changed(self, value: int) -> None:
        if self._updates_blocked:
            return
        self._slider.setValue(value)
        self.layerChanged.emit(value)


class GCodeScene(QGraphicsScene):
    """Scene that draws layer segments and nodes."""

    gcodeChanged = pyqtSignal()
    animationStateChanged = pyqtSignal(bool)
    animationProgressChanged = pyqtSignal(float)
    animationPlayingChanged = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.model: Optional[GCodeModel] = None
        self.current_layer: Optional[int] = None
        self.handles: Dict[int, DraggableHandle] = {}
        self.anchor_items: List[QGraphicsItem] = []
        self.segment_items: List[SegmentItem] = []
        self.extrusion_color = QColor(220, 90, 40)
        self.travel_color = QColor(70, 130, 200)
        self.anchor_color = QColor(210, 210, 210)

        self.animation_segments: List[GCodeSegment] = []
        self.animation_lengths: List[float] = []
        self.animation_prefix: List[float] = []
        self.animation_total_length: float = 0.0
        self.animation_timer = QTimer(self)
        self.animation_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.animation_timer.setInterval(16)
        self.animation_timer.timeout.connect(self._tick_animation)
        self.animation_speed = 40.0  # mm/sec
        self.animation_loop = True
        self.animation_running = False
        self.animation_active = False
        self.animation_path_item: Optional[QGraphicsPathItem] = None
        self.animation_segment_item: Optional[QGraphicsLineItem] = None
        self.animation_head_item: Optional[QGraphicsEllipseItem] = None
        self.animation_path = QPainterPath()
        self.animation_index = 0
        self.animation_progress = 0.0
        self.animation_last_time = 0.0
        self.animation_visuals_hidden = False

    def set_model(self, model: Optional[GCodeModel]) -> None:
        self.stop_animation(emit=False)
        self.model = model
        self.current_layer = None
        self.clear()
        self.handles.clear()
        self.anchor_items.clear()
        self.segment_items.clear()
        self.animation_segments = []
        self.animation_lengths = []
        self.animation_prefix = []
        self.animation_total_length = 0.0
        self.animation_active = False

    def set_layer(self, layer: int) -> None:
        if self.model is None:
            return
        self.stop_animation(emit=False)
        self.current_layer = layer
        self._rebuild()

    def _rebuild(self) -> None:
        self.stop_animation(emit=False)
        self.clear()
        self.handles.clear()
        self.anchor_items.clear()
        self.segment_items.clear()
        self.animation_segments = []
        self.setBackgroundBrush(QColor(35, 35, 35))
        if self.model is None or self.current_layer is None:
            return

        nodes = self.model.iter_nodes(self.current_layer)
        for node in nodes:
            color = QColor(255, 160, 0) if node.is_extrusion else QColor(120, 170, 255)
            handle = DraggableHandle(node, color, self._handle_commit)
            self.addItem(handle)
            self.handles[node.command_index] = handle

        segments = self.model.iter_segments(self.current_layer)
        segment_items: list[SegmentItem] = []
        for segment in segments:
            color = self.extrusion_color if segment.is_extrusion else self.travel_color
            item = SegmentItem(segment, color)
            start_point = QPointF(*segment.start)
            end_point = QPointF(*segment.end)

            # Attach handles if available
            end_handle = self.handles.get(segment.end_command_index)
            if end_handle:
                item.set_end_handle(end_handle)
            else:
                item.set_static_end(end_point)

            start_handle = self._find_previous_handle(segment.end_command_index)
            if start_handle:
                item.set_start_handle(start_handle)
            else:
                item.set_static_start(start_point)
                anchor = StaticAnchorItem(start_point)
                self.addItem(anchor)
                self.anchor_items.append(anchor)

            item.refresh()
            self.addItem(item)
            segment_items.append(item)

        self.segment_items = segment_items
        self.animation_segments = segments
        self._prepare_animation_data()
        self._emit_progress()

        bounds = self.model.get_bounds(self.current_layer)
        if bounds:
            min_x, max_x, min_y, max_y = bounds
            width = max_x - min_x
            height = max_y - min_y
            margin = max(width, height) * 0.05 + 2.0
            rect = (
                min_x - margin,
                min_y - margin,
                width + margin * 2.0,
                height + margin * 2.0,
            )
            self.setSceneRect(*rect)
        else:
            self.setSceneRect(-10, -10, 20, 20)

    def _handle_commit(self, handle: DraggableHandle, position: QPointF) -> None:
        if self.model is None or self.current_layer is None:
            return
        self.stop_animation()
        self.model.update_command_position(handle.node.command_index, position.x(), position.y())
        self._rebuild()
        self.gcodeChanged.emit()

    def _find_previous_handle(self, command_index: int) -> Optional[DraggableHandle]:
        if self.model is None:
            return None
        idx = command_index - 1
        while idx >= 0:
            candidate = self.model.commands[idx]
            if candidate.is_move() and candidate.layer == self.current_layer:
                handle = self.handles.get(idx)
                if handle:
                    return handle
            if candidate.layer is not None and candidate.layer < self.current_layer:
                break
            idx -= 1
        return None

    def _prepare_animation_data(self) -> None:
        lengths: List[float] = []
        prefix: List[float] = []
        total = 0.0
        for segment in self.animation_segments:
            length = self._segment_length(segment)
            lengths.append(length)
            total += max(length, 0.0)
            prefix.append(total)
        self.animation_lengths = lengths
        self.animation_prefix = prefix
        self.animation_total_length = total

    def start_animation(self) -> bool:
        if not self.animation_segments:
            self.stop_animation()
            return False

        self.stop_animation(emit=False)
        if not self._prepare_animation_items():
            self.stop_animation()
            return False

        if not self._reset_animation_state():
            self.stop_animation()
            return False

        self._set_items_for_animation(True)
        self.animation_active = True
        self.animation_running = True
        self.animation_last_time = time.perf_counter()
        self.animation_timer.start()
        self.animationPlayingChanged.emit(True)
        self.animationStateChanged.emit(True)
        self._emit_progress()
        return True

    def stop_animation(self, emit: bool = True) -> None:
        if self.animation_timer.isActive():
            self.animation_timer.stop()

        running = self.animation_running
        was_active = self.animation_active
        self.animation_running = False
        self.animation_active = False
        self.animation_last_time = 0.0
        self.animation_index = 0
        self.animation_progress = 0.0
        self.animation_path = QPainterPath()

        if self.animation_head_item is not None:
            self.removeItem(self.animation_head_item)
            self.animation_head_item = None
        if self.animation_segment_item is not None:
            self.removeItem(self.animation_segment_item)
            self.animation_segment_item = None
        if self.animation_path_item is not None:
            self.removeItem(self.animation_path_item)
            self.animation_path_item = None

        if emit:
            self.animationPlayingChanged.emit(False)
            if was_active:
                self.animationStateChanged.emit(False)
        self._set_items_for_animation(False)
        self._emit_progress()

    def _prepare_animation_items(self) -> bool:
        self.animation_path_item = QGraphicsPathItem()
        path_pen = QPen(QColor(255, 255, 255, 120))
        path_pen.setCosmetic(True)
        path_pen.setWidthF(0.5)
        path_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        self.animation_path_item.setPen(path_pen)
        self.animation_path_item.setZValue(3)
        self.addItem(self.animation_path_item)

        self.animation_segment_item = QGraphicsLineItem()
        self.animation_segment_item.setZValue(4)
        self.addItem(self.animation_segment_item)

        radius = 1.2
        self.animation_head_item = QGraphicsEllipseItem(-radius, -radius, radius * 2.0, radius * 2.0)
        self.animation_head_item.setPen(QPen(Qt.PenStyle.NoPen))
        self.animation_head_item.setBrush(QColor(255, 240, 180))
        self.animation_head_item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.animation_head_item.setZValue(5)
        self.addItem(self.animation_head_item)

        return True

    def _reset_animation_state(self) -> bool:
        self.animation_path = QPainterPath()
        next_index = self._find_next_segment_index(0)
        if next_index is None:
            return False
        self.animation_index = next_index
        self.animation_progress = 0.0

        segment = self.animation_segments[self.animation_index]
        start_point = QPointF(*segment.start)
        self.animation_path.moveTo(start_point)
        self.animation_path_item.setPath(self.animation_path)
        self._update_active_segment_pen(segment)
        self.animation_segment_item.setLine(start_point.x(), start_point.y(), start_point.x(), start_point.y())
        self.animation_head_item.setPos(start_point)
        self.animation_last_time = time.perf_counter()
        self._emit_progress()
        return True

    def _tick_animation(self) -> None:
        if not self.animation_running or not self.animation_segments:
            return

        now = time.perf_counter()
        if self.animation_last_time == 0.0:
            self.animation_last_time = now
            return

        dt = now - self.animation_last_time
        self.animation_last_time = now
        distance = max(dt, 0) * self.animation_speed
        self._advance_animation(distance)

    def _advance_animation(self, distance: float) -> None:
        remaining = distance
        while remaining > 0 and self.animation_index < len(self.animation_segments):
            segment = self.animation_segments[self.animation_index]
            length = self._segment_length(segment)
            if length <= 1e-6:
                if not self._advance_to_next_segment():
                    return
                continue

            start_point = QPointF(*segment.start)
            end_point = QPointF(*segment.end)
            remaining_length = (1.0 - self.animation_progress) * length

            if remaining >= remaining_length:
                # Finish segment.
                self.animation_path.lineTo(end_point)
                self.animation_path_item.setPath(self.animation_path)
                self.animation_segment_item.setLine(start_point.x(), start_point.y(), end_point.x(), end_point.y())
                self.animation_head_item.setPos(end_point)
                remaining -= remaining_length
                if not self._advance_to_next_segment():
                    return
            else:
                # Partial progress on current segment.
                self.animation_progress += remaining / length
                current_point = QPointF(
                    start_point.x() + (end_point.x() - start_point.x()) * self.animation_progress,
                    start_point.y() + (end_point.y() - start_point.y()) * self.animation_progress,
                )
                self.animation_segment_item.setLine(start_point.x(), start_point.y(), current_point.x(), current_point.y())
                self.animation_head_item.setPos(current_point)
                if self.animation_path_item is not None:
                    temp_path = QPainterPath(self.animation_path)
                    temp_path.lineTo(current_point)
                    self.animation_path_item.setPath(temp_path)
                remaining = 0.0
        self._emit_progress()

    def _advance_to_next_segment(self) -> bool:
        self.animation_index += 1
        next_index = self._find_next_segment_index(self.animation_index)
        if next_index is None:
            if self.animation_loop:
                return self._reset_animation_state()
            self.stop_animation()
            return False

        self.animation_index = next_index
        self.animation_progress = 0.0

        segment = self.animation_segments[self.animation_index]
        start_point = QPointF(*segment.start)
        current_pos = self.animation_path.currentPosition()
        if not self._points_close(current_pos, start_point):
            self.animation_path.moveTo(start_point)
            self.animation_path_item.setPath(self.animation_path)

        self._update_active_segment_pen(segment)
        self.animation_segment_item.setLine(start_point.x(), start_point.y(), start_point.x(), start_point.y())
        self.animation_head_item.setPos(start_point)
        return True

    def _update_active_segment_pen(self, segment: GCodeSegment) -> None:
        color = QColor(255, 190, 80) if segment.is_extrusion else QColor(150, 210, 255)
        pen = QPen(color)
        pen.setCosmetic(True)
        pen.setWidthF(1.0 if segment.is_extrusion else 0.8)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        self.animation_segment_item.setPen(pen)

        head_color = QColor(255, 220, 130) if segment.is_extrusion else QColor(190, 230, 255)
        self.animation_head_item.setBrush(head_color)

    @staticmethod
    def _segment_length(segment: GCodeSegment) -> float:
        return math.hypot(segment.end[0] - segment.start[0], segment.end[1] - segment.start[1])

    @staticmethod
    def _points_close(a: QPointF, b: QPointF, eps: float = 1e-5) -> bool:
        return abs(a.x() - b.x()) <= eps and abs(a.y() - b.y()) <= eps

    def _find_next_segment_index(self, start_index: int) -> Optional[int]:
        for idx in range(start_index, len(self.animation_segments)):
            if self._segment_length(self.animation_segments[idx]) > 1e-6:
                return idx
        return None

    def _find_last_segment_index(self) -> Optional[int]:
        for idx in range(len(self.animation_segments) - 1, -1, -1):
            if self._segment_length(self.animation_segments[idx]) > 1e-6:
                return idx
        return None

    def set_animation_speed(self, speed: float) -> None:
        self.animation_speed = max(0.1, float(speed))
        if self.animation_running:
            self.animation_last_time = time.perf_counter()

    def pause_animation(self) -> None:
        if not self.animation_active or not self.animation_running:
            return
        if self.animation_timer.isActive():
            self.animation_timer.stop()
        self.animation_running = False
        self.animation_last_time = 0.0
        self.animationPlayingChanged.emit(False)

    def resume_animation(self) -> bool:
        if not self.animation_active or self.animation_running:
            return False
        self.animation_running = True
        self.animation_last_time = time.perf_counter()
        self.animation_timer.start()
        self.animationPlayingChanged.emit(True)
        return True

    def set_animation_fraction(self, fraction: float) -> None:
        if not self.animation_segments:
            return
        fraction = max(0.0, min(fraction, 1.0))
        if (
            self.animation_path_item is None
            or self.animation_segment_item is None
            or self.animation_head_item is None
        ):
            if not self._prepare_animation_items():
                return
        self._set_items_for_animation(True)
        self.animation_active = True
        distance = fraction * self.animation_total_length if self.animation_total_length > 0.0 else 0.0
        self._set_state_for_distance(distance)

    def _set_state_for_distance(self, distance: float) -> None:
        if not self.animation_segments:
            return
        if (
            self.animation_path_item is None
            or self.animation_segment_item is None
            or self.animation_head_item is None
        ):
            return

        if self.animation_total_length <= 1e-6:
            index = self._find_next_segment_index(0)
            if index is None:
                return
            segment = self.animation_segments[index]
            start_point = QPointF(*segment.start)
            self.animation_path = QPainterPath()
            self.animation_path.moveTo(start_point)
            self.animation_path_item.setPath(self.animation_path)
            self._update_active_segment_pen(segment)
            self.animation_segment_item.setLine(start_point.x(), start_point.y(), start_point.x(), start_point.y())
            self.animation_head_item.setPos(start_point)
            self.animation_index = index
            self.animation_progress = 0.0
            self.animation_last_time = time.perf_counter()
            self._emit_progress()
            return

        distance = max(0.0, min(distance, self.animation_total_length))
        path = QPainterPath()
        current_total = 0.0

        for idx, segment in enumerate(self.animation_segments):
            start_point = QPointF(*segment.start)
            end_point = QPointF(*segment.end)
            length = self.animation_lengths[idx] if idx < len(self.animation_lengths) else self._segment_length(segment)
            if path.isEmpty():
                path.moveTo(start_point)
            elif not self._points_close(path.currentPosition(), start_point):
                path.moveTo(start_point)

            if length <= 1e-6:
                continue

            if distance >= current_total + length:
                path.lineTo(end_point)
                current_total += length
                self.animation_index = idx + 1
                self.animation_progress = 0.0
                continue

            ratio = (distance - current_total) / length if length > 0 else 0.0
            ratio = max(0.0, min(ratio, 1.0))
            current_point = QPointF(
                start_point.x() + (end_point.x() - start_point.x()) * ratio,
                start_point.y() + (end_point.y() - start_point.y()) * ratio,
            )
            if ratio > 0.0:
                path.lineTo(current_point)
            self.animation_path = path
            self.animation_path_item.setPath(path)
            self._update_active_segment_pen(segment)
            self.animation_segment_item.setLine(start_point.x(), start_point.y(), current_point.x(), current_point.y())
            self.animation_head_item.setPos(current_point)
            self.animation_index = idx
            self.animation_progress = ratio
            self.animation_last_time = time.perf_counter()
            self._emit_progress()
            return

        last_index = self._find_last_segment_index()
        if last_index is None:
            return
        last_segment = self.animation_segments[last_index]
        start_point = QPointF(*last_segment.start)
        end_point = QPointF(*last_segment.end)
        if path.isEmpty():
            path.moveTo(start_point)
        if self._segment_length(last_segment) > 1e-6:
            path.lineTo(end_point)
        self.animation_path = path
        self.animation_path_item.setPath(path)
        self._update_active_segment_pen(last_segment)
        self.animation_segment_item.setLine(start_point.x(), start_point.y(), end_point.x(), end_point.y())
        self.animation_head_item.setPos(end_point)
        self.animation_index = last_index
        self.animation_progress = 1.0
        self.animation_last_time = time.perf_counter()
        self._emit_progress()

    def _current_travel_distance(self) -> float:
        if not self.animation_segments:
            return 0.0
        completed = 0.0
        if self.animation_index > 0 and self.animation_index - 1 < len(self.animation_prefix):
            completed = self.animation_prefix[self.animation_index - 1]
        current = 0.0
        if 0 <= self.animation_index < len(self.animation_lengths):
            current = self.animation_lengths[self.animation_index] * self.animation_progress
        return completed + current

    def _emit_progress(self) -> None:
        if not self.animation_segments or self.animation_total_length <= 1e-6:
            progress = 0.0
        else:
            progress = min(
                max(self._current_travel_distance() / self.animation_total_length, 0.0),
                1.0,
            )
        self.animationProgressChanged.emit(progress)

    def _set_items_for_animation(self, active: bool) -> None:
        if self.animation_visuals_hidden == active:
            return
        visible = not active
        for item in self.segment_items:
            item.setVisible(visible)
        for handle in self.handles.values():
            handle.setVisible(visible)
        for anchor in self.anchor_items:
            anchor.setVisible(visible)
        self.animation_visuals_hidden = active


class GraphicsView(QGraphicsView):
    """Custom view with inverted Y axis and wheel zoom."""

    def __init__(self, scene: QGraphicsScene):
        super().__init__(scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        # Flip Y axis to match typical printer coordinate system.
        self.scale(1, -1)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setBackgroundBrush(QColor(30, 30, 30))

    def wheelEvent(self, event):
        angle = event.angleDelta().y()
        if angle == 0:
            return
        zoom_factor = 1.15 if angle > 0 else 1 / 1.15
        self.scale(zoom_factor, zoom_factor)

    def fit_to_rect(self, rect: QRectF) -> None:
        if not rect.isValid():
            return
        self.resetTransform()
        super().fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)
        super().scale(1, -1)
        self.centerOn(rect.center())


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("G-code Fixer")
        self.resize(1200, 800)

        self.model: Optional[GCodeModel] = None
        self.current_path: Optional[Path] = None
        self._need_auto_fit: bool = False
        self._block_animation_toggle: bool = False
        self._block_speed_updates: bool = False
        self._block_timeline_updates: bool = False
        self._timeline_was_running: bool = False
        self._timeline_dragging: bool = False

        central = QWidget(self)
        self.setCentralWidget(central)

        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        controls = QHBoxLayout()
        controls.setSpacing(6)

        self.open_button = QPushButton("Открыть G-code…", self)
        self.open_button.clicked.connect(self.open_file)

        self.save_button = QPushButton("Сохранить как…", self)
        self.save_button.clicked.connect(self.save_file_as)
        self.save_button.setEnabled(False)

        self.layer_selector = LayerSelector(self)
        self.layer_selector.layerChanged.connect(self._on_layer_changed)

        controls.addWidget(self.open_button)
        controls.addWidget(self.save_button)
        controls.addStretch(1)
        controls.addWidget(self.layer_selector)

        self.animate_button = QPushButton("Анимация слоя", self)
        self.animate_button.setCheckable(True)
        self.animate_button.setEnabled(False)
        self.animate_button.toggled.connect(self._toggle_animation)

        self.play_pause_button = QPushButton("Пауза", self)
        self.play_pause_button.setEnabled(False)
        self.play_pause_button.clicked.connect(self._on_play_pause_clicked)

        self.speed_label = QLabel("Скорость: 40 мм/с", self)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal, self)
        self.speed_slider.setRange(5, 200)
        self.speed_slider.setSingleStep(1)
        self.speed_slider.setPageStep(5)
        self.speed_slider.setValue(40)
        self.speed_slider.setEnabled(False)
        self.speed_slider.valueChanged.connect(self._on_speed_changed)

        self.timeline_label = QLabel("Прогресс: 0%", self)
        self.timeline_label.setEnabled(False)

        self.timeline_slider = QSlider(Qt.Orientation.Horizontal, self)
        self.timeline_slider.setRange(0, 1000)
        self.timeline_slider.setSingleStep(1)
        self.timeline_slider.setPageStep(25)
        self.timeline_slider.setValue(0)
        self.timeline_slider.setEnabled(False)
        self.timeline_slider.sliderPressed.connect(self._on_timeline_pressed)
        self.timeline_slider.sliderReleased.connect(self._on_timeline_released)
        self.timeline_slider.valueChanged.connect(self._on_timeline_value_changed)

        animation_controls = QHBoxLayout()
        animation_controls.setSpacing(6)
        animation_controls.addWidget(self.animate_button)
        animation_controls.addWidget(self.play_pause_button)
        animation_controls.addWidget(self.speed_label)
        animation_controls.addWidget(self.speed_slider, 1)
        animation_controls.addWidget(self.timeline_label)
        animation_controls.addWidget(self.timeline_slider, 2)

        self.scene = GCodeScene()
        self.scene.gcodeChanged.connect(self._refresh_code_view)
        self.scene.animationStateChanged.connect(self._on_animation_state_changed)
        self.scene.animationPlayingChanged.connect(self._on_animation_playing_changed)
        self.scene.animationProgressChanged.connect(self._on_animation_progress)
        self.scene.set_animation_speed(self.speed_slider.value())
        self.view = GraphicsView(self.scene)
        self.code_view = QPlainTextEdit(self)
        self.code_view.setReadOnly(True)
        self.code_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        fixed_font = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
        self.code_view.setFont(fixed_font)
        self.code_view.setMinimumWidth(260)
        self.code_view.setPlaceholderText("Здесь будет исходный G-code после загрузки файла.")

        self.splitter = QSplitter(Qt.Orientation.Horizontal, self)
        self.splitter.addWidget(self.view)
        self.splitter.addWidget(self.code_view)
        self.splitter.setStretchFactor(0, 3)
        self.splitter.setStretchFactor(1, 2)

        layout.addLayout(controls)
        layout.addLayout(animation_controls)
        layout.addWidget(self.splitter, 1)

        self.status_label = QLabel("Загрузите файл G-code.", self)
        layout.addWidget(self.status_label)

    def open_file(self) -> None:
        dialog = QFileDialog(self)
        dialog.setFileMode(QFileDialog.FileMode.ExistingFile)
        dialog.setNameFilter("G-code (*.gcode *.gco *.gc *.nc);;Все файлы (*)")
        if dialog.exec() == QFileDialog.DialogCode.Accepted:
            file_path = Path(dialog.selectedFiles()[0])
            try:
                with file_path.open("r", encoding="utf-8", errors="ignore") as fh:
                    lines = fh.readlines()
            except OSError as exc:
                QMessageBox.critical(self, "Ошибка", f"Не удалось открыть файл:\n{exc}")
                return

            self.model = GCodeModel(lines)
            self.scene.set_model(self.model)
            layer_count = self.model.get_layer_count()
            self._block_animation_toggle = True
            self.animate_button.setChecked(False)
            self._block_animation_toggle = False
            self.animate_button.setEnabled(layer_count > 0)
            self._set_speed_controls_enabled(False)
            self._set_timeline_controls_enabled(False)
            self._update_play_pause_button(False)
            self.timeline_slider.setValue(0)
            self.timeline_label.setText("Прогресс: 0%")
            self.layer_selector.set_layer_count(layer_count)
            self.save_button.setEnabled(True)
            self.current_path = file_path
            self._need_auto_fit = True
            self.status_label.setText(f"Загружен файл: {file_path.name} ({layer_count} слоев)")
            self._refresh_code_view()
            self._on_layer_changed(0)

    def save_file_as(self) -> None:
        if self.model is None:
            return
        default_dir = str(self.current_path.parent) if self.current_path else os.getcwd()
        dialog = QFileDialog(self)
        dialog.setAcceptMode(QFileDialog.AcceptMode.AcceptSave)
        dialog.setDirectory(default_dir)
        dialog.setNameFilter("G-code (*.gcode *.gco *.gc *.nc);;Все файлы (*)")
        if self.current_path:
            dialog.selectFile(self.current_path.name)
        if dialog.exec() != QFileDialog.DialogCode.Accepted:
            return

        target_path = Path(dialog.selectedFiles()[0])
        try:
            content = "\n".join(self.model.rebuild())
            target_path.write_text(content, encoding="utf-8")
        except OSError as exc:
            QMessageBox.critical(self, "Ошибка", f"Не удалось сохранить файл:\n{exc}")
            return
        self.statusBar().showMessage(f"Файл сохранен: {target_path}", 5000)

    def _on_layer_changed(self, layer: int) -> None:
        if self.model is None:
            return
        self.layer_selector.set_layer(layer)
        self.scene.set_layer(layer)
        has_segments = bool(self.scene.animation_segments)
        if not has_segments and self.animate_button.isChecked():
            self._block_animation_toggle = True
            self.animate_button.setChecked(False)
            self._block_animation_toggle = False
        self.animate_button.setEnabled(has_segments)
        self._set_speed_controls_enabled(has_segments)
        if self.animate_button.isChecked():
            self.scene.set_animation_speed(self.speed_slider.value())
            if not self.scene.start_animation():
                self._block_animation_toggle = True
                self.animate_button.setChecked(False)
                self._block_animation_toggle = False
                self._set_timeline_controls_enabled(False)
                self._update_play_pause_button(False)
            else:
                self._set_timeline_controls_enabled(True)
        else:
            self._set_timeline_controls_enabled(False)
            self._update_play_pause_button(False)
            self.timeline_slider.setValue(0)
            self.timeline_label.setText("Прогресс: 0%")
        if self._need_auto_fit:
            self.view.fit_to_rect(self.scene.sceneRect())
            self._need_auto_fit = False
        bounds = self.model.get_bounds(layer)
        if bounds:
            min_x, max_x, min_y, max_y = bounds
            center = QPointF((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
            self.view.centerOn(center)
        total_layers = self.model.get_layer_count()
        file_name = self.current_path.name if self.current_path else "G-code"
        self.status_label.setText(f"{file_name}: слой {layer + 1} из {max(total_layers, 1)}")

    def _refresh_code_view(self) -> None:
        if self.model is None:
            self.code_view.clear()
            return
        scrollbar = self.code_view.verticalScrollBar()
        value = scrollbar.value()
        content = "\n".join(self.model.rebuild())
        self.code_view.setPlainText(content)
        scrollbar.setValue(min(value, scrollbar.maximum()))

    def _set_speed_controls_enabled(self, enabled: bool) -> None:
        self.speed_slider.setEnabled(enabled)
        self.speed_label.setEnabled(enabled)

    def _set_timeline_controls_enabled(self, enabled: bool) -> None:
        self.timeline_slider.setEnabled(enabled)
        self.timeline_label.setEnabled(enabled)
        self.play_pause_button.setEnabled(enabled)
        if not enabled:
            self.timeline_slider.setValue(0)
            self.timeline_label.setText("Прогресс: 0%")

    def _toggle_animation(self, checked: bool) -> None:
        if self._block_animation_toggle:
            return
        if checked:
            self.scene.set_animation_speed(self.speed_slider.value())
            if not self.scene.start_animation():
                self._block_animation_toggle = True
                self.animate_button.setChecked(False)
                self._block_animation_toggle = False
                self.statusBar().showMessage("��� ���������� ��� �������� ����� ����.", 4000)
                self._set_timeline_controls_enabled(False)
                self._update_play_pause_button(False)
            else:
                self._set_timeline_controls_enabled(True)
                self._update_play_pause_button(True)
                self.timeline_slider.setValue(0)
                self.timeline_label.setText("Прогресс: 0%")
        else:
            self.scene.stop_animation()
            self._set_timeline_controls_enabled(False)
            self._update_play_pause_button(False)

    def _on_speed_changed(self, value: int) -> None:
        if self._block_speed_updates:
            return
        self.speed_label.setText(f"Скорость: {value} мм/с")
        self.scene.set_animation_speed(value)

    def _on_animation_state_changed(self, running: bool) -> None:
        if self._block_animation_toggle:
            return
        if self.animate_button.isChecked() != running:
            self._block_animation_toggle = True
            self.animate_button.setChecked(running)
            self._block_animation_toggle = False
        if not running and self.statusBar():
            self.statusBar().showMessage("Анимация остановлена.", 2000)


def main() -> None:
    import sys

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


__all__ = ["MainWindow", "main"]
