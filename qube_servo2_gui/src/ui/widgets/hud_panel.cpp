#include "qube_servo2_gui/ui/widgets/hud_panel.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPalette>

#include <algorithm>

namespace qube_servo2::gui::ui::widgets {

HudPanel::HudPanel(const QString& title, QWidget* parent)
    : QFrame(parent), title_(title) {
    setObjectName("hudPanel");
    setFrameShape(QFrame::NoFrame);
    setAttribute(Qt::WA_StyledBackground, true);
}

void HudPanel::setHudTitle(const QString& title) {
    if (title_ == title) return;
    title_ = title;
    update();
}

QString HudPanel::hudTitle() const {
    return title_;
}

void HudPanel::paintEvent(QPaintEvent* event) {
    QFrame::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF r = rect().adjusted(1.0, 1.0, -1.0, -1.0);
    if (r.width() < 30.0 || r.height() < 30.0) return;

    const qreal cut = 12.0;
    QPainterPath path;
    path.moveTo(r.left() + cut, r.top());
    path.lineTo(r.right() - 26.0, r.top());
    path.lineTo(r.right(), r.top() + 26.0);
    path.lineTo(r.right(), r.bottom() - cut);
    path.lineTo(r.right() - cut, r.bottom());
    path.lineTo(r.left() + 24.0, r.bottom());
    path.lineTo(r.left(), r.bottom() - 24.0);
    path.lineTo(r.left(), r.top() + cut);
    path.closeSubpath();

    QColor accent = palette().color(QPalette::Highlight);
    QColor panel = palette().color(QPalette::Base);
    QColor text = palette().color(QPalette::WindowText);
    QColor muted = palette().color(QPalette::PlaceholderText);
    if (!muted.isValid()) muted = text.darker(170);

    QColor fill = panel;
    fill.setAlpha(232);
    QColor border = accent;
    border.setAlpha(150);

    painter.setPen(QPen(border, 1.1));
    painter.setBrush(fill);
    painter.drawPath(path);

    QColor glow = accent;
    glow.setAlpha(70);
    painter.setPen(QPen(glow, 3.0));
    painter.drawLine(QPointF(r.left() + cut + 1.0, r.top() + 1.0),
                     QPointF(r.left() + 74.0, r.top() + 1.0));

    if (!title_.isEmpty()) {
        QFont font = painter.font();
        font.setBold(true);
        font.setPointSizeF(std::max<qreal>(9.0, font.pointSizeF()));
        font.setLetterSpacing(QFont::AbsoluteSpacing, 0.8);
        painter.setFont(font);
        painter.setPen(accent.lighter(135));
        painter.drawText(QRectF(r.left() + 12.0, r.top() + 4.0, r.width() - 24.0, 18.0),
                         Qt::AlignLeft | Qt::AlignVCenter,
                         title_.toUpper());

        painter.setPen(QPen(border, 0.8));
        painter.drawLine(QPointF(r.left() + 10.0, r.top() + 24.0),
                         QPointF(r.right() - 10.0, r.top() + 24.0));
    }

    painter.setPen(QPen(border, 1.0));
    painter.drawLine(QPointF(r.right() - 28.0, r.top() + 4.0),
                     QPointF(r.right() - 8.0, r.top() + 24.0));
    painter.drawLine(QPointF(r.left() + 4.0, r.bottom() - 26.0),
                     QPointF(r.left() + 22.0, r.bottom() - 8.0));
}

}  // namespace qube_servo2::gui::ui::widgets
