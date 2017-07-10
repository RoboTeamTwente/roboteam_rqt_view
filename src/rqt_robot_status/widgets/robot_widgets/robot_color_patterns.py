

from python_qt_binding.QtGui import QColor, QImage, QPainter
from python_qt_binding.QtCore import Qt


WHITE = QColor(255, 255, 255)
GREEN = QColor(0, 255, 0)
BLUE = QColor(0, 0, 255)
CYAN = QColor(255, 0, 255)
YELLOW = QColor(255, 255, 0)
ROBOT_BODY = QColor(0, 0, 0)
BACKGROUND = QColor(0, 0, 0, 0)


"""
Patterns are defined as follows:
 Front

 X -> X
      V
  X<-X

 Back
"""
PATTERNS = {
    -1: [WHITE, WHITE, WHITE, WHITE], # For when a given id doesn't have a pattern.
    0: [CYAN, CYAN, CYAN, GREEN],
    1: [GREEN, CYAN, CYAN, GREEN],
    2: [GREEN, GREEN, CYAN, GREEN],
    3: [CYAN, GREEN, CYAN, GREEN],
    4: [CYAN, CYAN, GREEN, CYAN],
    5: [GREEN, CYAN, GREEN, CYAN],
    6: [GREEN, GREEN, GREEN, CYAN],
    7: [CYAN, GREEN, GREEN, CYAN],
    8: [GREEN, GREEN, GREEN, GREEN],
    9: [CYAN, CYAN, CYAN, CYAN],
    10: [CYAN, CYAN, GREEN, GREEN],
    11: [GREEN, GREEN, CYAN, CYAN]
}


def get_pattern(bot_id):
    if bot_id in PATTERNS:
        return PATTERNS[bot_id]
    else:
        return PATTERNS[-1]


def draw_pattern_image(bot_id, size):
    image = QImage(size, size, QImage.Format_ARGB32)
    image.fill(BACKGROUND)

    painter = QPainter(image)
    painter.setCompositionMode(QPainter.CompositionMode_Source)
    painter.setRenderHint(QPainter.Antialiasing, True)

    painter.setPen(Qt.NoPen)
    painter.setBrush(ROBOT_BODY)
    painter.drawEllipse(0, 0, size, size)

    painter.fillRect(0, 0, size, size * 0.15, BACKGROUND)

    dot_size = size * 0.2

    pattern = get_pattern(bot_id)

    painter.setBrush(pattern[0])
    painter.drawEllipse(size * 0.1, size * 0.2, dot_size, dot_size)
    painter.setBrush(pattern[1])
    painter.drawEllipse((size * 0.9) - dot_size, size * 0.2, dot_size, dot_size)
    painter.setBrush(pattern[2])
    painter.drawEllipse(size * 0.2, (size * 0.9) - dot_size, dot_size, dot_size)
    painter.setBrush(pattern[3])
    painter.drawEllipse((size * 0.8) - dot_size, (size * 0.9) - dot_size, dot_size, dot_size)

    return image
