from __future__ import annotations

import math
import os
import time
from bisect import bisect_left
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from enum import Enum
from datetime import datetime

try:
    from PyQt6 import sip as _sip
except ImportError:  # pragma: no cover
    _sip = None
from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal, QSize
from PyQt6.QtGui import QColor, QFontDatabase, QPainter, QPainterPath, QPen, QBrush, QTransform, QKeySequence, QFont, QIcon, QAction
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
    QStyle,
    QStyleOptionGraphicsItem,
    QMenu,
    QGraphicsTextItem,
    QCheckBox,
    QToolBar,
    QButtonGroup,
    QGroupBox,
    QFormLayout,
    QLineEdit,
    QDoubleSpinBox,
)

from .gcode_parser import GCodeModel, GCodeNode, GCodeSegment


class EditMode(Enum):
    """Editing modes for the CAD-like editor."""
    SELECT = "select"
    DRAW_EXTRUSION = "draw_extrusion"
    DRAW_TRAVEL = "draw_travel"
    DELETE = "delete"
    MEASURE = "measure"


class AngleItem(QGraphicsItem):
    """Visual angle indicator between two segments."""

    def __init__(self, vertex: QPointF, angle_deg: float, radius: float = 5.0):
        super().__init__()
        self.vertex = vertex
        self.angle_deg = angle_deg
        self.radius = radius
        self.text_item: Optional[QGraphicsTextItem] = None
        self.setPos(vertex)
        self.setZValue(16)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self._create_text()

    def _create_text(self):
        if self.text_item is None and self.scene():
            self.text_item = QGraphicsTextItem()
            font = QFont("Arial", 9, QFont.Weight.Bold)
            self.text_item.setFont(font)
            self.text_item.setDefaultTextColor(QColor(100, 255, 100))
            self.text_item.setZValue(17)
            self.text_item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
            self.scene().addItem(self.text_item)

        if self.text_item:
            self.text_item.setPlainText(f"{self.angle_deg:.1f}°")
            self.text_item.setPos(self.vertex.x() + 8, self.vertex.y() + 8)

    def boundingRect(self) -> QRectF:
        return QRectF(-self.radius, -self.radius, self.radius * 2, self.radius * 2)

    def paint(self, painter: QPainter, option, widget=None):
        # Draw small arc at vertex
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = QPen(QColor(100, 255, 100, 200), 0)
        pen.setCosmetic(True)
        painter.setPen(pen)
        # Simple visual indicator
        painter.drawEllipse(QPointF(0, 0), 2, 2)

    def remove_from_scene(self):
        if self.text_item and self.text_item.scene():
            self.text_item.scene().removeItem(self.text_item)
        self.text_item = None


class DimensionItem(QGraphicsLineItem):
    """Visual dimension line showing distance between two points."""

    def __init__(self, start: QPointF, end: QPointF):
        super().__init__()
        self.start_point = start
        self.end_point = end
        self.text_item: Optional[QGraphicsTextItem] = None
        self.update_geometry()

        # Style the dimension line
        pen = QPen(QColor(255, 255, 100, 200), 0)
        pen.setCosmetic(True)
        pen.setStyle(Qt.PenStyle.DashLine)
        self.setPen(pen)
        self.setZValue(15)

    def update_geometry(self) -> None:
        """Update the line and text based on start and end points."""
        self.setLine(self.start_point.x(), self.start_point.y(),
                     self.end_point.x(), self.end_point.y())

        # Calculate distance
        dx = self.end_point.x() - self.start_point.x()
        dy = self.end_point.y() - self.start_point.y()
        distance = math.hypot(dx, dy)

        # Update or create text item
        if self.text_item is None and self.scene():
            self.text_item = QGraphicsTextItem()
            font = QFont("Arial", 8)
            self.text_item.setFont(font)
            self.text_item.setDefaultTextColor(QColor(255, 255, 150))
            self.text_item.setZValue(16)
            self.text_item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
            self.scene().addItem(self.text_item)

        if self.text_item:
            # Position text at midpoint
            mid_x = (self.start_point.x() + self.end_point.x()) / 2
            mid_y = (self.start_point.y() + self.end_point.y()) / 2

            # Calculate angle
            angle = math.degrees(math.atan2(dy, dx))

            self.text_item.setPlainText(f"{distance:.2f} мм")
            self.text_item.setPos(mid_x, mid_y)

    def set_points(self, start: QPointF, end: QPointF) -> None:
        """Update the dimension with new points."""
        self.start_point = start
        self.end_point = end
        self.update_geometry()

    def remove_from_scene(self) -> None:
        """Clean up text item when removing from scene."""
        if self.text_item and self.text_item.scene():
            self.text_item.scene().removeItem(self.text_item)
        self.text_item = None


class StaticAnchorItem(QGraphicsEllipseItem):
    """Visual marker for a fixed (non-draggable) point."""

    def __init__(self, position: QPointF, radius: float = 0.4):
        super().__init__(-radius, -radius, radius * 2.0, radius * 2.0)
        self.setPos(position)
        self.setPen(QPen(QColor(180, 180, 180), 0))
        self._pending_creation_start_index: Optional[int] = None
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
        self._base_color = QColor(color)
        pen = QPen(self._base_color, 0.4)  # Scene units width for better visibility
        pen.setCosmetic(False)  # Use scene coordinates, not screen pixels
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        self.setPen(pen)
        self.setZValue(1)  # Поднял над фоном
        self.setAcceptHoverEvents(True)
        self._active_handle: Optional["DraggableHandle"] = None
        self._hovered = False
        self.setFlags(
            QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
            | QGraphicsItem.GraphicsItemFlag.ItemIsFocusable
        )
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        # Ensure visible by default
        self.setVisible(True)

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget=None):
        """Override paint to ensure lines are always visible."""
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return

        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setPen(self.pen())

        line = self.line()
        if line.length() > 0:
            painter.drawLine(line)

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
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return
        start = self.start_handle.scenePos() if self.start_handle else self.static_start
        end = self.end_handle.scenePos() if self.end_handle else self.static_end
        if start and end:
            self.setLine(start.x(), start.y(), end.x(), end.y())
        self._update_pen()

    def _update_pen(self) -> None:
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return
        color = QColor(self._base_color)
        width = 0.4  # Scene units width

        if self._active_handle:
            color = color.lighter(150)
            width = 0.6
        elif self.isSelected():
            # CAD-style selection: bright cyan highlight
            color = QColor(0, 255, 255)
            width = 0.8
        elif self._hovered:
            color = color.lighter(140)
            width = 0.5

        pen = QPen(color, width)
        pen.setCosmetic(False)  # Use scene coordinates
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        if self.isSelected():
            pen.setStyle(Qt.PenStyle.SolidLine)
        self.setPen(pen)

    def hoverEnterEvent(self, event) -> None:
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return
        self._hovered = True
        self._update_pen()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event) -> None:
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return
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

            # Multi-selection support for segments
            modifiers = event.modifiers()
            if not (modifiers & Qt.KeyboardModifier.ControlModifier):
                # Clear other selections if Ctrl is not pressed
                scene = self.scene()
                if scene and not self.isSelected():
                    scene.clearSelection()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return
        if self._active_handle:
            self._active_handle.setPos(event.scenePos())
            for segment, _ in list(self._active_handle._segments):
                segment.refresh()
            # Real-time dimension/angle updates during drag
            scene = self.scene()
            if isinstance(scene, GCodeScene):
                scene.update_dimensions()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return
        if self._active_handle:
            handle = self._active_handle
            self._active_handle = None
            if not (_sip and _sip.isdeleted(handle)):
                handle.setPos(event.scenePos())
                for segment, _ in list(handle._segments):
                    segment.refresh()
                if handle.commit_callback:
                    handle.commit_callback(handle, handle.scenePos())
            self._update_pen()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            scene = self.scene()
            if isinstance(scene, GCodeScene) and scene.split_segment_at(self, event.scenePos()):
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def contextMenuEvent(self, event) -> None:
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return
        scene = self.scene()
        if not isinstance(scene, GCodeScene):
            return

        menu = QMenu()
        split_action = menu.addAction("Разделить сегмент в этой точке")
        delete_action = menu.addAction("Удалить сегмент")
        menu.addSeparator()

        if self.segment.is_extrusion:
            convert_action = menu.addAction("Преобразовать в travel")
        else:
            convert_action = menu.addAction("Преобразовать в экструзию")

        # Show menu at cursor position
        for view in scene.views():
            action = menu.exec(event.screenPos())
            if not (_sip and _sip.isdeleted(self)):
                if action == split_action:
                    scene.split_segment_at(self, event.scenePos())
                elif action == delete_action:
                    scene.delete_segment(self)
                elif action == convert_action:
                    scene.convert_segment(self)
            break

        event.accept()

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if (_sip and _sip.isdeleted(self)) or self.scene() is None:
            return super().itemChange(change, value)
        if change in {
            QGraphicsItem.GraphicsItemChange.ItemSelectedChange,
            QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged,
        }:
            self._update_pen()
            # Update dimensions when selection changes
            scene = self.scene()
            if isinstance(scene, GCodeScene):
                scene.update_dimensions()
        return super().itemChange(change, value)


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
        self._base_color = QColor(color)
        self._hovered = False
        self.setBrush(QBrush(self._base_color))
        outline = QPen(QColor(40, 40, 40), 0)
        outline.setCosmetic(True)
        self.setPen(outline)
        self.setFlags(
            QGraphicsItem.GraphicsItemFlag.ItemIsMovable
            | QGraphicsItem.GraphicsItemFlag.ItemSendsScenePositionChanges
            | QGraphicsItem.GraphicsItemFlag.ItemIsSelectable
        )
        self.setZValue(10)
        self.setPos(QPointF(*node.position))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def attach_segment(self, segment: SegmentItem, is_start: bool) -> None:
        self._segments.append((segment, is_start))

    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if _sip and _sip.isdeleted(self):
            return super().itemChange(change, value)
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for segment, _ in self._segments:
                segment.refresh()
            # Update dimensions when handle moves
            scene = self.scene()
            if isinstance(scene, GCodeScene):
                scene.update_dimensions()
        if change in {
            QGraphicsItem.GraphicsItemChange.ItemSelectedChange,
            QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged,
        }:
            self.update()
            # Update dimensions when selection changes
            scene = self.scene()
            if isinstance(scene, GCodeScene):
                scene.update_dimensions()
        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        if _sip and _sip.isdeleted(self):
            return
        scene = self.scene()
        if (
            event.button() == Qt.MouseButton.LeftButton
            and isinstance(scene, GCodeScene)
            and scene.creation_mode == "travel"
        ):
            if scene.handle_creation_click(self):
                event.accept()
                return

        # Multi-selection support
        if event.button() == Qt.MouseButton.LeftButton:
            modifiers = event.modifiers()
            if not (modifiers & Qt.KeyboardModifier.ControlModifier):
                # Clear other selections if Ctrl is not pressed
                if scene and not self.isSelected():
                    scene.clearSelection()

        self.setCursor(Qt.CursorShape.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """Handle mouse move events for real-time dimension updates."""
        super().mouseMoveEvent(event)
        # Real-time dimension/angle updates during drag
        if not (_sip and _sip.isdeleted(self)):
            scene = self.scene()
            if isinstance(scene, GCodeScene):
                scene.update_dimensions()

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        # Check if object is still valid
        if _sip and _sip.isdeleted(self):
            return
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        if self.commit_callback:
            position = self.scenePos()
            self.commit_callback(self, position)
        # Check again after callback (rebuild might have deleted this)
        if not (_sip and _sip.isdeleted(self)):
            self.update()

    def hoverEnterEvent(self, event):
        if _sip and _sip.isdeleted(self):
            return
        self._hovered = True
        self.update()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        if _sip and _sip.isdeleted(self):
            return
        self._hovered = False
        self.update()
        super().hoverLeaveEvent(event)

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget=None):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        color = QColor(self._base_color)
        outline = QColor(40, 40, 40)
        outline_width = 0

        if option.state & QStyle.StateFlag.State_Selected:
            # CAD-style selection: bright cyan with glow effect
            outline = QColor(0, 255, 255)
            outline_width = 2.0
            # Draw outer glow
            glow_pen = QPen(QColor(0, 255, 255, 100), 4.0)
            glow_pen.setCosmetic(True)
            painter.setPen(glow_pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(self.rect())
        elif self._hovered or option.state & QStyle.StateFlag.State_MouseOver:
            outline = color.lighter(150)
            outline_width = 1.5

        # Draw main circle
        painter.setBrush(QBrush(color))
        pen = QPen(outline, outline_width)
        pen.setCosmetic(True)
        painter.setPen(pen)
        painter.drawEllipse(self.rect())

        if option.state & QStyle.StateFlag.State_Selected:
            # Draw center cross for selected handles
            painter.setPen(QPen(QColor(0, 255, 255), 0))
            rect = self.rect()
            center_x = rect.center().x()
            center_y = rect.center().y()
            size = rect.width() * 0.4
            painter.drawLine(QPointF(center_x - size, center_y), QPointF(center_x + size, center_y))
            painter.drawLine(QPointF(center_x, center_y - size), QPointF(center_x, center_y + size))

    def contextMenuEvent(self, event) -> None:
        if _sip and _sip.isdeleted(self):
            return
        scene = self.scene()
        if not isinstance(scene, GCodeScene):
            return

        menu = QMenu()
        delete_action = menu.addAction("Удалить точку")
        menu.addSeparator()

        if self.node.is_extrusion:
            convert_action = menu.addAction("Преобразовать в travel")
        else:
            convert_action = menu.addAction("Преобразовать в экструзию")

        # Show menu at cursor position
        for view in scene.views():
            action = menu.exec(event.screenPos())
            if not (_sip and _sip.isdeleted(self)):
                if action == delete_action:
                    scene.delete_handle(self)
                elif action == convert_action:
                    scene.convert_handle(self)
            break

        event.accept()


class PropertiesPanel(QWidget):
    """Panel showing properties of selected items."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Title
        title = QLabel("<b>Свойства выделенного</b>", self)
        layout.addWidget(title)

        # Properties form
        form_widget = QWidget(self)
        self.form_layout = QFormLayout(form_widget)
        self.form_layout.setContentsMargins(0, 0, 0, 0)
        self.form_layout.setSpacing(4)

        # Coordinate fields
        self.x_coord = QLineEdit(self)
        self.x_coord.setReadOnly(True)
        self.y_coord = QLineEdit(self)
        self.y_coord.setReadOnly(True)

        # Distance field
        self.distance = QLineEdit(self)
        self.distance.setReadOnly(True)

        # Angle field
        self.angle = QLineEdit(self)
        self.angle.setReadOnly(True)

        # Type field
        self.item_type = QLineEdit(self)
        self.item_type.setReadOnly(True)

        # Command index
        self.cmd_index = QLineEdit(self)
        self.cmd_index.setReadOnly(True)

        self.form_layout.addRow("X:", self.x_coord)
        self.form_layout.addRow("Y:", self.y_coord)
        self.form_layout.addRow("Длина:", self.distance)
        self.form_layout.addRow("Угол:", self.angle)
        self.form_layout.addRow("Тип:", self.item_type)
        self.form_layout.addRow("Индекс:", self.cmd_index)

        layout.addWidget(form_widget)
        layout.addStretch()

        self.setMaximumWidth(250)
        self.setMinimumWidth(200)

    def update_selection(self, selected_items: List):
        """Update properties based on selected items."""
        if not selected_items:
            self.clear()
            return

        if len(selected_items) == 1:
            item = selected_items[0]
            if isinstance(item, DraggableHandle):
                self._show_handle_properties(item)
            elif isinstance(item, SegmentItem):
                self._show_segment_properties(item)
        else:
            self._show_multiple_selection(selected_items)

    def _show_handle_properties(self, handle: DraggableHandle):
        pos = handle.scenePos()
        self.x_coord.setText(f"{pos.x():.3f} мм")
        self.y_coord.setText(f"{pos.y():.3f} мм")
        self.distance.setText("-")
        self.angle.setText("-")
        self.item_type.setText("Экструзия" if handle.node.is_extrusion else "Travel")
        self.cmd_index.setText(f"{handle.node.command_index}")

    def _show_segment_properties(self, segment: SegmentItem):
        line = segment.line()
        start = QPointF(line.x1(), line.y1())
        end = QPointF(line.x2(), line.y2())
        length = math.hypot(end.x() - start.x(), end.y() - start.y())
        angle_rad = math.atan2(end.y() - start.y(), end.x() - start.x())
        angle_deg = math.degrees(angle_rad)

        self.x_coord.setText(f"({start.x():.2f}, {start.y():.2f})")
        self.y_coord.setText(f"({end.x():.2f}, {end.y():.2f})")
        self.distance.setText(f"{length:.3f} мм")
        self.angle.setText(f"{angle_deg:.1f}°")
        self.item_type.setText("Экструзия" if segment.segment.is_extrusion else "Travel")
        self.cmd_index.setText(f"{segment.segment.end_command_index}")

    def _show_multiple_selection(self, items: List):
        count = len(items)
        self.x_coord.setText(f"Выбрано: {count}")
        self.y_coord.setText("-")
        self.distance.setText("-")
        self.angle.setText("-")
        self.item_type.setText("Несколько")
        self.cmd_index.setText("-")

    def clear(self):
        """Clear all fields."""
        self.x_coord.setText("-")
        self.y_coord.setText("-")
        self.distance.setText("-")
        self.angle.setText("-")
        self.item_type.setText("-")
        self.cmd_index.setText("-")


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
        self.dimension_items: List[DimensionItem] = []
        self.angle_items: List[AngleItem] = []
        self.show_dimensions: bool = True
        self.show_angles: bool = True
        self.snap_to_grid: bool = False
        self.grid_size: float = 1.0  # 1mm grid
        self.edit_mode: EditMode = EditMode.SELECT
        self.drawing_points: List[QPointF] = []
        self.drawing_preview: Optional[QGraphicsLineItem] = None
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
        self.creation_mode: Optional[str] = None
        self.creation_start_handle: Optional[DraggableHandle] = None
        self.creation_preview: Optional[QGraphicsLineItem] = None
        self._pending_creation_start_index: Optional[int] = None

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
        self.creation_mode = None
        self.creation_start_handle = None
        if self.creation_preview is not None:
            self.removeItem(self.creation_preview)
            self.creation_preview = None
        self._pending_creation_start_index = None

    def set_layer(self, layer: int) -> None:
        if self.model is None:
            return
        self.stop_animation(emit=False)
        self.current_layer = layer
        self._rebuild()

    def _rebuild(self) -> None:
        prev_start_index = None
        if self.creation_mode == "travel" and self.creation_start_handle is not None:
            prev_start_index = self.creation_start_handle.node.command_index
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
        if self.creation_mode == "travel":
            target_index = (
                self._pending_creation_start_index
                if self._pending_creation_start_index is not None
                else prev_start_index
            )
            if target_index is not None:
                handle = self.handles.get(target_index)
                if handle:
                    self.creation_start_handle = handle
                    self.clearSelection()
                    handle.setSelected(True)
                    self._update_creation_preview(handle.scenePos(), handle.scenePos())
                else:
                    self.creation_start_handle = None
                    self._clear_creation_preview()
            else:
                self.creation_start_handle = None
                self._clear_creation_preview()
        else:
            self.creation_start_handle = None
            self._clear_creation_preview()
        self._pending_creation_start_index = None

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

        # Ensure all items are visible if not animating
        if not self.animation_active:
            for item in self.segment_items:
                item.setVisible(True)
            for handle in self.handles.values():
                handle.setVisible(True)
            for anchor in self.anchor_items:
                anchor.setVisible(True)

    def _handle_commit(self, handle: DraggableHandle, position: QPointF) -> None:
        if self.model is None or self.current_layer is None:
            return
        self.stop_animation()

        # Apply grid snapping if enabled
        if self.snap_to_grid:
            position = self._snap_to_grid(position)

        self.model.update_command_position(handle.node.command_index, position.x(), position.y())
        self._rebuild()
        self.gcodeChanged.emit()

    def _snap_to_grid(self, point: QPointF) -> QPointF:
        """Snap a point to the nearest grid intersection."""
        if not self.snap_to_grid or self.grid_size <= 0:
            return point
        x = round(point.x() / self.grid_size) * self.grid_size
        y = round(point.y() / self.grid_size) * self.grid_size
        return QPointF(x, y)

    def set_snap_to_grid(self, enabled: bool) -> None:
        """Enable or disable grid snapping."""
        self.snap_to_grid = enabled
        if enabled:
            self._notify(f'Привязка к сетке включена (шаг {self.grid_size} мм).', 2500)
        else:
            self._notify('Привязка к сетке отключена.', 2500)

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

        self.set_creation_mode(None)
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

    def set_creation_mode(self, mode: Optional[str]) -> None:
        if mode == self.creation_mode:
            return
        if mode is None:
            if self.creation_start_handle:
                self.creation_start_handle.setSelected(False)
            self.creation_mode = None
            self.creation_start_handle = None
            self._pending_creation_start_index = None
            self._clear_creation_preview()
            return
        if mode == "travel":
            self.creation_mode = mode
            self.creation_start_handle = None
            self._pending_creation_start_index = None
            self._clear_creation_preview()
            self._set_items_for_animation(False)
        else:
            self.creation_mode = None
            self.creation_start_handle = None
            self._pending_creation_start_index = None
            self._clear_creation_preview()

    def handle_creation_click(self, handle: DraggableHandle) -> bool:
        if self.creation_mode != "travel":
            return False
        if self.creation_start_handle is None:
            self.creation_start_handle = handle
            self.clearSelection()
            handle.setSelected(True)
            self._update_creation_preview(handle.scenePos(), handle.scenePos())
            self._notify('Выберите конечную точку travel-линии или нажмите на поле, чтобы задать новую.', 3500)
            return True
        if handle is self.creation_start_handle:
            handle.setSelected(False)
            self.creation_start_handle = None
            self._clear_creation_preview()
            return True
        return self._create_travel_move(self.creation_start_handle, handle.scenePos(), handle)

    def split_segment_at(self, segment_item: SegmentItem, position: QPointF) -> bool:
        if self.model is None:
            return False
        if segment_item.segment.is_extrusion:
            self._notify('Экструзионные сегменты пока нельзя разделить.', 3000)
            return False
        inserted = self.model.split_travel_command(
            segment_item.segment.end_command_index, (position.x(), position.y())
        )
        if inserted is None:
            self._notify('Не удалось разделить выбранный сегмент.', 3000)
            return False
        if self.creation_mode == 'travel':
            self._pending_creation_start_index = inserted
        else:
            self._pending_creation_start_index = None
        self.creation_start_handle = None
        self._clear_creation_preview()
        self._rebuild()
        self.gcodeChanged.emit()
        self._notify('Сегмент разделён.', 2500)
        return True

    def _create_travel_move(
        self, start_handle: DraggableHandle, target_pos: QPointF, target_handle: Optional[DraggableHandle]
    ) -> bool:
        if self.model is None:
            return False
        target_point = QPointF(target_pos)
        if target_handle is not None:
            target_point = target_handle.scenePos()
        start_point = start_handle.scenePos()
        if math.hypot(target_point.x() - start_point.x(), target_point.y() - start_point.y()) < 1e-6:
            self._notify('Стартовая и конечная точки совпадают.', 2500)
            return False
        try:
            inserted_index = self.model.insert_travel_move(
                start_handle.node.command_index, (target_point.x(), target_point.y())
            )
        except (IndexError, ValueError):
            self._notify('Не удалось добавить travel-линию.', 3000)
            return False
        if self.creation_mode == 'travel':
            if target_handle is not None:
                self._pending_creation_start_index = target_handle.node.command_index
            else:
                self._pending_creation_start_index = inserted_index
        else:
            self._pending_creation_start_index = None
        self.creation_start_handle = None
        self._clear_creation_preview()
        self._rebuild()
        self.gcodeChanged.emit()
        self._notify('Добавлена travel-линия.', 2500)
        return True

    def _update_creation_preview(self, start_point: Optional[QPointF], end_point: Optional[QPointF]) -> None:
        if self.creation_mode != 'travel' or start_point is None:
            self._clear_creation_preview()
            return
        if end_point is None:
            end_point = start_point

        # Check if preview was deleted
        if self.creation_preview is not None and (_sip and _sip.isdeleted(self.creation_preview)):
            self.creation_preview = None

        if self.creation_preview is None:
            preview = QGraphicsLineItem()
            pen = QPen(QColor(200, 200, 200, 160), 0)
            pen.setCosmetic(True)
            pen.setStyle(Qt.PenStyle.DashLine)
            preview.setPen(pen)
            preview.setZValue(2)
            self.addItem(preview)
            self.creation_preview = preview

        if not (_sip and _sip.isdeleted(self.creation_preview)):
            self.creation_preview.setLine(
                start_point.x(), start_point.y(), end_point.x(), end_point.y()
            )

    def _clear_creation_preview(self) -> None:
        if self.creation_preview is not None:
            if not (_sip and _sip.isdeleted(self.creation_preview)):
                try:
                    self.removeItem(self.creation_preview)
                except RuntimeError:
                    pass  # Already deleted
            self.creation_preview = None

    def _notify(self, message: str, timeout: int = 3000) -> None:
        for view in self.views():
            window = view.window()
            if hasattr(window, 'statusBar'):
                try:
                    window.statusBar().showMessage(message, timeout)
                except Exception:  # pragma: no cover - UI guard
                    pass
                break

    def mousePressEvent(self, event):
        if (
            self.creation_mode == 'travel'
            and event.button() == Qt.MouseButton.LeftButton
            and self.creation_start_handle is not None
        ):
            transform = self.views()[0].transform() if self.views() else QTransform()
            item = self.itemAt(event.scenePos(), transform)
            if not isinstance(item, DraggableHandle):
                if self._create_travel_move(self.creation_start_handle, event.scenePos(), None):
                    event.accept()
                    return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.creation_mode == 'travel' and self.creation_start_handle is not None:
            self._update_creation_preview(self.creation_start_handle.scenePos(), event.scenePos())
        super().mouseMoveEvent(event)

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

        # Only hide during animation, not in edit mode
        if active:
            for item in self.segment_items:
                item.setVisible(visible)
            for handle in self.handles.values():
                handle.setVisible(visible)
            for anchor in self.anchor_items:
                anchor.setVisible(visible)
            # Hide dimensions during animation
            for dim in self.dimension_items:
                dim.setVisible(visible)
        else:
            # Show everything when not animating
            for item in self.segment_items:
                item.setVisible(True)
            for handle in self.handles.values():
                handle.setVisible(True)
            for anchor in self.anchor_items:
                anchor.setVisible(True)

        self.animation_visuals_hidden = active

    def update_dimensions(self) -> None:
        """Update dimension lines and angles for selected items."""
        # Clear existing dimensions and angles
        for dim in self.dimension_items:
            dim.remove_from_scene()
            if dim.scene():
                self.removeItem(dim)
        self.dimension_items.clear()

        for angle_item in self.angle_items:
            angle_item.remove_from_scene()
            if angle_item.scene():
                self.removeItem(angle_item)
        self.angle_items.clear()

        if not self.show_dimensions and not self.show_angles:
            return

        # Get all selected items
        selected = self.selectedItems()
        selected_segments = [item for item in selected if isinstance(item, SegmentItem)]
        selected_handles = [item for item in selected if isinstance(item, DraggableHandle)]

        # Show dimensions for selected segments
        if self.show_dimensions:
            for segment_item in selected_segments:
                start = QPointF(segment_item.line().x1(), segment_item.line().y1())
                end = QPointF(segment_item.line().x2(), segment_item.line().y2())
                dim = DimensionItem(start, end)
                self.addItem(dim)
                self.dimension_items.append(dim)

            # Show dimensions between consecutive selected handles
            if len(selected_handles) >= 2:
                # Sort handles by command index
                sorted_handles = sorted(selected_handles, key=lambda h: h.node.command_index)
                for i in range(len(sorted_handles) - 1):
                    start = sorted_handles[i].scenePos()
                    end = sorted_handles[i + 1].scenePos()
                    dim = DimensionItem(start, end)
                    self.addItem(dim)
                    self.dimension_items.append(dim)

        # Show angles between consecutive segments
        if self.show_angles and len(selected_handles) >= 3:
            sorted_handles = sorted(selected_handles, key=lambda h: h.node.command_index)
            for i in range(1, len(sorted_handles) - 1):
                prev_pos = sorted_handles[i - 1].scenePos()
                curr_pos = sorted_handles[i].scenePos()
                next_pos = sorted_handles[i + 1].scenePos()

                # Calculate angle
                angle = self._calculate_angle(prev_pos, curr_pos, next_pos)
                if angle is not None:
                    angle_item = AngleItem(curr_pos, angle)
                    self.addItem(angle_item)
                    self.angle_items.append(angle_item)

    def _calculate_angle(self, p1: QPointF, vertex: QPointF, p2: QPointF) -> Optional[float]:
        """Calculate angle at vertex between three points in degrees."""
        # Vector from vertex to p1
        v1_x = p1.x() - vertex.x()
        v1_y = p1.y() - vertex.y()

        # Vector from vertex to p2
        v2_x = p2.x() - vertex.x()
        v2_y = p2.y() - vertex.y()

        # Calculate lengths
        len1 = math.hypot(v1_x, v1_y)
        len2 = math.hypot(v2_x, v2_y)

        if len1 < 1e-6 or len2 < 1e-6:
            return None

        # Normalize vectors
        v1_x /= len1
        v1_y /= len1
        v2_x /= len2
        v2_y /= len2

        # Calculate dot product
        dot = v1_x * v2_x + v1_y * v2_y
        dot = max(-1.0, min(1.0, dot))  # Clamp to avoid numerical errors

        # Calculate angle in degrees
        angle_rad = math.acos(dot)
        angle_deg = math.degrees(angle_rad)

        return angle_deg

    def toggle_dimensions(self, show: bool) -> None:
        """Toggle dimension display on/off."""
        self.show_dimensions = show
        if not show:
            for dim in self.dimension_items:
                dim.remove_from_scene()
                if dim.scene():
                    self.removeItem(dim)
            self.dimension_items.clear()
        else:
            self.update_dimensions()

    def delete_segment(self, segment_item: SegmentItem) -> bool:
        """Delete a segment by removing its end command."""
        if self.model is None:
            return False
        try:
            self.model.delete_command(segment_item.segment.end_command_index)
            self._rebuild()
            self.gcodeChanged.emit()
            self._notify('Сегмент удалён.', 2500)
            return True
        except (IndexError, ValueError) as e:
            self._notify(f'Не удалось удалить сегмент: {e}', 3000)
            return False

    def convert_segment(self, segment_item: SegmentItem) -> bool:
        """Convert segment between extrusion and travel."""
        if self.model is None:
            return False
        try:
            self.model.toggle_extrusion(segment_item.segment.end_command_index)
            self._rebuild()
            self.gcodeChanged.emit()
            seg_type = "travel" if segment_item.segment.is_extrusion else "экструзию"
            self._notify(f'Сегмент преобразован в {seg_type}.', 2500)
            return True
        except (IndexError, ValueError) as e:
            self._notify(f'Не удалось преобразовать сегмент: {e}', 3000)
            return False

    def delete_handle(self, handle: DraggableHandle) -> bool:
        """Delete a point (handle) by removing its command."""
        if self.model is None:
            return False
        try:
            self.model.delete_command(handle.node.command_index)
            self._rebuild()
            self.gcodeChanged.emit()
            self._notify('Точка удалена.', 2500)
            return True
        except (IndexError, ValueError) as e:
            self._notify(f'Не удалось удалить точку: {e}', 3000)
            return False

    def convert_handle(self, handle: DraggableHandle) -> bool:
        """Convert handle command between extrusion and travel."""
        if self.model is None:
            return False
        try:
            self.model.toggle_extrusion(handle.node.command_index)
            self._rebuild()
            self.gcodeChanged.emit()
            cmd_type = "travel" if handle.node.is_extrusion else "экструзию"
            self._notify(f'Команда преобразована в {cmd_type}.', 2500)
            return True
        except (IndexError, ValueError) as e:
            self._notify(f'Не удалось преобразовать команду: {e}', 3000)
            return False

    def delete_selected_items(self) -> bool:
        """Delete all selected segments and handles."""
        if self.model is None:
            return False

        selected = self.selectedItems()
        selected_segments = [item for item in selected if isinstance(item, SegmentItem)]
        selected_handles = [item for item in selected if isinstance(item, DraggableHandle)]

        # Get indices to delete (avoid duplicates)
        indices_to_delete = set()
        for segment in selected_segments:
            indices_to_delete.add(segment.segment.end_command_index)
        for handle in selected_handles:
            indices_to_delete.add(handle.node.command_index)

        if not indices_to_delete:
            return False

        # Sort in reverse order to delete from end to start (preserves indices)
        sorted_indices = sorted(indices_to_delete, reverse=True)

        deleted_count = 0
        for idx in sorted_indices:
            try:
                self.model.delete_command(idx)
                deleted_count += 1
            except (IndexError, ValueError):
                pass

        if deleted_count > 0:
            self._rebuild()
            self.gcodeChanged.emit()
            self._notify(f'Удалено элементов: {deleted_count}', 2500)
            return True

        return False


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
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts."""
        scene = self.scene()
        if not isinstance(scene, GCodeScene):
            super().keyPressEvent(event)
            return

        key = event.key()
        modifiers = event.modifiers()

        # Delete key: delete selected items
        if key in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            selected = scene.selectedItems()
            if selected:
                scene.delete_selected_items()
                event.accept()
                return

        # Escape: deselect all
        elif key == Qt.Key.Key_Escape:
            scene.clearSelection()
            event.accept()
            return

        # Ctrl+A: select all
        elif key == Qt.Key.Key_A and (modifiers & Qt.KeyboardModifier.ControlModifier):
            for item in scene.items():
                if isinstance(item, (SegmentItem, DraggableHandle)):
                    item.setSelected(True)
            event.accept()
            return

        # D key: toggle dimensions
        elif key == Qt.Key.Key_D and not (modifiers & Qt.KeyboardModifier.ControlModifier):
            scene.toggle_dimensions(not scene.show_dimensions)
            event.accept()
            return

        # G key: toggle grid snap
        elif key == Qt.Key.Key_G:
            scene.set_snap_to_grid(not scene.snap_to_grid)
            event.accept()
            return

        super().keyPressEvent(event)

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
        self.setWindowTitle("G-code Fixer - CAD Редактор")
        self.resize(1400, 900)

        self.model: Optional[GCodeModel] = None
        self.current_path: Optional[Path] = None
        self._need_auto_fit: bool = False
        self._block_animation_toggle: bool = False
        self._block_speed_updates: bool = False
        self._block_timeline_updates: bool = False
        self._block_creation_toggle: bool = False
        self._timeline_was_running: bool = False
        self._timeline_dragging: bool = False

        # Create toolbar
        self._create_toolbar()

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

        self.create_travel_button = QPushButton("Новая travel-линия", self)
        self.create_travel_button.setCheckable(True)
        self.create_travel_button.setEnabled(False)
        self.create_travel_button.toggled.connect(self._on_create_travel_toggled)

        self.show_dimensions_checkbox = QCheckBox("Показать размеры", self)
        self.show_dimensions_checkbox.setChecked(True)
        self.show_dimensions_checkbox.toggled.connect(self._on_dimensions_toggled)

        self.snap_to_grid_checkbox = QCheckBox("Привязка к сетке", self)
        self.snap_to_grid_checkbox.setChecked(False)
        self.snap_to_grid_checkbox.toggled.connect(self._on_snap_toggled)

        self.play_pause_button = QPushButton("Пауза", self)
        self.play_pause_button.setEnabled(False)
        self.play_pause_button.clicked.connect(self._on_play_pause_clicked)
        self._update_play_pause_button(False)

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
        animation_controls.addWidget(self.create_travel_button)
        animation_controls.addWidget(self.show_dimensions_checkbox)
        animation_controls.addWidget(self.snap_to_grid_checkbox)
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
        self.scene.selectionChanged.connect(self._on_selection_changed)
        self.scene.set_animation_speed(self.speed_slider.value())
        self.view = GraphicsView(self.scene)

        # Properties panel
        self.properties_panel = PropertiesPanel(self)

        self.code_view = QPlainTextEdit(self)
        self.code_view.setReadOnly(True)
        self.code_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        fixed_font = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
        self.code_view.setFont(fixed_font)
        self.code_view.setPlaceholderText("Здесь будет исходный G-code после загрузки файла.")

        self.splitter = QSplitter(Qt.Orientation.Horizontal, self)
        self.splitter.addWidget(self.properties_panel)
        self.splitter.addWidget(self.view)
        self.splitter.addWidget(self.code_view)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 4)
        self.splitter.setStretchFactor(2, 2)

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
            self.status_label.setText(f"Загружен файл: {file_path.name} ({layer_count} слоёв)")
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
        self.statusBar().showMessage(f"Файл сохранён: {target_path}", 5000)

    def _on_layer_changed(self, layer: int) -> None:
        if self.model is None:
            return
        self.layer_selector.set_layer(layer)
        self.scene.set_layer(layer)
        has_segments = bool(self.scene.animation_segments)
        if not has_segments and self.create_travel_button.isChecked():
            self._block_creation_toggle = True
            self.create_travel_button.setChecked(False)
            self._block_creation_toggle = False
        self.create_travel_button.setEnabled(has_segments and not self.animate_button.isChecked())
        if not has_segments and self.animate_button.isChecked():
            self._block_animation_toggle = True
            self.animate_button.setChecked(False)
            self._block_animation_toggle = False
        self.animate_button.setEnabled(has_segments)
        self._set_speed_controls_enabled(has_segments)
        if self.animate_button.isChecked():
            self.create_travel_button.setEnabled(False)
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

    def _update_play_pause_button(self, running: bool) -> None:
        self.play_pause_button.setText("Пауза" if running else "Продолжить")

    def _on_play_pause_clicked(self) -> None:
        if not self.scene.animation_active:
            return
        if self.scene.animation_running:
            self.scene.pause_animation()
        else:
            if not self.scene.resume_animation():
                self._update_play_pause_button(False)



    def _on_animation_progress(self, fraction: float) -> None:
        percent = max(0.0, min(fraction, 1.0)) * 100.0
        self.timeline_label.setText(f"Прогресс: {percent:.1f}%")
        if self._block_timeline_updates:
            return
        max_value = max(self.timeline_slider.maximum(), 1)
        value = int(round(fraction * max_value))
        self._block_timeline_updates = True
        self.timeline_slider.setValue(value)
        self._block_timeline_updates = False

    def _on_animation_playing_changed(self, running: bool) -> None:
        self._update_play_pause_button(running)

    def _on_timeline_value_changed(self, value: int) -> None:
        max_value = max(self.timeline_slider.maximum(), 1)
        fraction = value / max_value
        self.timeline_label.setText(f"Прогресс: {fraction * 100:.1f}%")
        if self._block_timeline_updates or not self.animate_button.isChecked():
            return
        self._block_timeline_updates = True
        self.scene.set_animation_fraction(fraction)
        self._block_timeline_updates = False

    def _on_timeline_pressed(self) -> None:
        if not self.animate_button.isChecked():
            return
        self._timeline_dragging = True
        self._timeline_was_running = self.scene.animation_running
        if self.scene.animation_running:
            self.scene.pause_animation()

    def _on_timeline_released(self) -> None:
        if not self.animate_button.isChecked():
            self._timeline_dragging = False
            self._timeline_was_running = False
            return
        self._timeline_dragging = False
        max_value = max(self.timeline_slider.maximum(), 1)
        fraction = self.timeline_slider.value() / max_value
        self._block_timeline_updates = True
        self.scene.set_animation_fraction(fraction)
        self._block_timeline_updates = False
        if self._timeline_was_running:
            self.scene.resume_animation()
        self._timeline_was_running = False

    def _on_create_travel_toggled(self, checked: bool) -> None:
        if self._block_creation_toggle:
            return
        if checked:
            if self.animate_button.isChecked():
                self.animate_button.setChecked(False)
            self.scene.set_creation_mode("travel")
            if self.statusBar():
                self.statusBar().showMessage("Выберите стартовую точку travel-линии.", 4000)
        else:
            self.scene.set_creation_mode(None)
            self.scene.clearSelection()
            if self.statusBar():
                self.statusBar().clearMessage()

    def _toggle_animation(self, checked: bool) -> None:
        if self._block_animation_toggle:
            return
        if checked:
            self._block_creation_toggle = True
            self.create_travel_button.setChecked(False)
            self._block_creation_toggle = False
            self.create_travel_button.setEnabled(False)
            self.scene.set_animation_speed(self.speed_slider.value())
            if not self.scene.start_animation():
                self._block_animation_toggle = True
                self.animate_button.setChecked(False)
                self._block_animation_toggle = False
                if self.statusBar():
                    self.statusBar().showMessage("Нет траекторий для анимации этого слоя.", 4000)
                self._set_timeline_controls_enabled(False)
                self._update_play_pause_button(False)
                self.create_travel_button.setEnabled(bool(self.scene.animation_segments))
            else:
                self._set_timeline_controls_enabled(True)
                self._update_play_pause_button(True)
                self.timeline_slider.setValue(0)
                self.timeline_label.setText("Прогресс: 0%")
        else:
            self.scene.stop_animation()
            self._set_timeline_controls_enabled(False)
            self._update_play_pause_button(False)
            self.create_travel_button.setEnabled(bool(self.scene.animation_segments))

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
        if running:
            self._block_creation_toggle = True
            self.create_travel_button.setChecked(False)
            self._block_creation_toggle = False
            self.create_travel_button.setEnabled(False)
            self._set_timeline_controls_enabled(True)
            self._update_play_pause_button(self.scene.animation_running)
        else:
            self._set_timeline_controls_enabled(False)
            self._update_play_pause_button(False)
            self.create_travel_button.setEnabled(bool(self.scene.animation_segments))
            if self.statusBar():
                self.statusBar().showMessage("Анимация остановлена.", 2000)

    def _on_dimensions_toggled(self, checked: bool) -> None:
        """Toggle dimension display."""
        self.scene.toggle_dimensions(checked)

    def _on_snap_toggled(self, checked: bool) -> None:
        """Toggle snap to grid."""
        self.scene.set_snap_to_grid(checked)

    def _create_toolbar(self):
        """Create CAD-style toolbar with mode buttons."""
        toolbar = QToolBar("Инструменты", self)
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(32, 32))
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, toolbar)

        # Mode button group (mutually exclusive)
        self.mode_group = QButtonGroup(self)
        self.mode_group.setExclusive(True)

        # Select mode
        self.select_action = QAction("✋ Выбор", self)
        self.select_action.setCheckable(True)
        self.select_action.setChecked(True)
        self.select_action.setToolTip("Режим выбора (S)")
        self.select_action.setShortcut(QKeySequence("S"))
        self.select_action.triggered.connect(lambda: self._set_edit_mode(EditMode.SELECT))
        toolbar.addAction(self.select_action)
        self.mode_group.addButton(toolbar.widgetForAction(self.select_action))

        # Draw Extrusion mode
        self.draw_extrusion_action = QAction("✏ Экструзия", self)
        self.draw_extrusion_action.setCheckable(True)
        self.draw_extrusion_action.setToolTip("Рисовать экструзию (E)")
        self.draw_extrusion_action.setShortcut(QKeySequence("E"))
        self.draw_extrusion_action.triggered.connect(lambda: self._set_edit_mode(EditMode.DRAW_EXTRUSION))
        toolbar.addAction(self.draw_extrusion_action)
        self.mode_group.addButton(toolbar.widgetForAction(self.draw_extrusion_action))

        # Draw Travel mode
        self.draw_travel_action = QAction("📍 Travel", self)
        self.draw_travel_action.setCheckable(True)
        self.draw_travel_action.setToolTip("Рисовать travel-линию (T)")
        self.draw_travel_action.setShortcut(QKeySequence("T"))
        self.draw_travel_action.triggered.connect(lambda: self._set_edit_mode(EditMode.DRAW_TRAVEL))
        toolbar.addAction(self.draw_travel_action)
        self.mode_group.addButton(toolbar.widgetForAction(self.draw_travel_action))

        toolbar.addSeparator()

        # Delete mode
        self.delete_action = QAction("🗑 Удалить", self)
        self.delete_action.setCheckable(True)
        self.delete_action.setToolTip("Удалить выделенное (Delete)")
        self.delete_action.triggered.connect(lambda: self.scene.delete_selected_items())
        toolbar.addAction(self.delete_action)

        toolbar.addSeparator()

        # Measure mode
        self.measure_action = QAction("📏 Измерить", self)
        self.measure_action.setCheckable(True)
        self.measure_action.setToolTip("Режим измерений (M)")
        self.measure_action.setShortcut(QKeySequence("M"))
        self.measure_action.triggered.connect(lambda: self._set_edit_mode(EditMode.MEASURE))
        toolbar.addAction(self.measure_action)
        self.mode_group.addButton(toolbar.widgetForAction(self.measure_action))

        toolbar.addSeparator()

        # View controls
        self.zoom_fit_action = QAction("🔍 По размеру", self)
        self.zoom_fit_action.setToolTip("Вписать в окно (F)")
        self.zoom_fit_action.setShortcut(QKeySequence("F"))
        self.zoom_fit_action.triggered.connect(lambda: self.view.fit_to_rect(self.scene.sceneRect()))
        toolbar.addAction(self.zoom_fit_action)

    def _set_edit_mode(self, mode: EditMode):
        """Change the editing mode."""
        self.scene.edit_mode = mode
        self.statusBar().showMessage(f"Режим: {mode.value}", 2000)

    def _on_selection_changed(self):
        """Update properties panel when selection changes."""
        selected = self.scene.selectedItems()
        self.properties_panel.update_selection(selected)

def main() -> None:
    import sys

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


__all__ = ["MainWindow", "main"]
