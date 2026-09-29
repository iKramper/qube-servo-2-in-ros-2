#include "qube_servo2_gui/ui/widgets/throttle_lever.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::ui::widgets {

ThrottleLever::ThrottleLever(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(150, 255);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    setCursor(Qt::SizeVerCursor);
    setFocusPolicy(Qt::StrongFocus);
}

void ThrottleLever::setRange(double minimum, double maximum) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum) return;
    minimum_ = minimum;
    maximum_ = maximum;
    setValue(value_);
    update();
}

void ThrottleLever::setValue(double value) {
    if (!std::isfinite(value)) return;
    const double clamped = std::clamp(value, minimum_, maximum_);
    if (std::abs(clamped - value_) < 1e-10) return;
    value_ = clamped;
    update();
    emit valueChanged(value_);
}

void ThrottleLever::setUnit(const QString& unit) {
    unit_ = unit;
    update();
}

void ThrottleLever::setCaption(const QString& caption) {
    caption_ = caption;
    update();
}

double ThrottleLever::value() const noexcept { return value_; }
double ThrottleLever::minimum() const noexcept { return minimum_; }
double ThrottleLever::maximum() const noexcept { return maximum_; }

double ThrottleLever::valueFromY_(qreal y) const noexcept {
    const qreal top = 38.0;
    const qreal bottom = height() - 42.0;
    const qreal h = std::max<qreal>(1.0, bottom - top);
    const double alpha = 1.0 - std::clamp((y - top) / h, 0.0, 1.0);
    return minimum_ + alpha * (maximum_ - minimum_);
}

qreal ThrottleLever::yFromValue_(double value) const noexcept {
    const qreal top = 38.0;
    const qreal bottom = height() - 42.0;
    const qreal h = std::max<qreal>(1.0, bottom - top);
    const double alpha = std::clamp((value - minimum_) / (maximum_ - minimum_), 0.0, 1.0);
    return top + (1.0 - alpha) * h;
}

void ThrottleLever::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        setValue(valueFromY_(event->position().y()));
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void ThrottleLever::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_ && (event->buttons() & Qt::LeftButton)) {
        setValue(valueFromY_(event->position().y()));
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void ThrottleLever::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = false;
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void ThrottleLever::wheelEvent(QWheelEvent* event) {
    const double step = (maximum_ - minimum_) / 120.0;
    if (event->angleDelta().y() > 0) setValue(value_ + step);
    else if (event->angleDelta().y() < 0) setValue(value_ - step);
    event->accept();
}

void ThrottleLever::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    const auto pal = palette();
    QColor accent = pal.color(QPalette::Highlight);
    QColor text = pal.color(QPalette::WindowText);
    QColor muted = pal.color(QPalette::PlaceholderText);
    if (!muted.isValid()) muted = text.darker(170);

    const QRectF outer = rect().adjusted(12.0, 8.0, -12.0, -8.0);
    QColor base = pal.color(QPalette::Base);
    base.setAlpha(225);
    QColor border = accent;
    border.setAlpha(120);
    p.setPen(QPen(border, 1.0));
    p.setBrush(base);
    p.drawRoundedRect(outer, 8.0, 8.0);

    const qreal x = width() * 0.50;
    const qreal top = 38.0;
    const qreal bottom = height() - 42.0;
    const QRectF slot(x - 13.0, top, 26.0, bottom - top);
    QColor slot_color = pal.color(QPalette::Window);
    slot_color = slot_color.darker(115);
    p.setPen(QPen(border.darker(145), 1.0));
    p.setBrush(slot_color);
    p.drawRoundedRect(slot, 8.0, 8.0);

    const qreal zero_y = yFromValue_(0.0);
    QColor zero = muted;
    zero.setAlpha(120);
    p.setPen(QPen(zero, 1.0, Qt::DashLine));
    p.drawLine(QPointF(outer.left() + 12.0, zero_y), QPointF(outer.right() - 12.0, zero_y));

    QFont tick_font("DejaVu Sans Mono", 7);
    p.setFont(tick_font);
    p.setPen(muted);
    for (int i = 0; i <= 8; ++i) {
        const double alpha = static_cast<double>(i) / 8.0;
        const double v = maximum_ - alpha * (maximum_ - minimum_);
        const qreal y = top + alpha * (bottom - top);
        const qreal tick = (i % 2 == 0) ? 12.0 : 7.0;
        p.drawLine(QPointF(slot.left() - tick, y), QPointF(slot.left() - 2.0, y));
        if (i % 2 == 0) {
            p.drawText(QRectF(outer.left() + 2.0, y - 7.0, slot.left() - outer.left() - 18.0, 14.0),
                       Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(v, 'f', 0));
        }
    }

    const qreal y = yFromValue_(value_);
    // Keep the handle compact so the control reads visually as an aircraft
    // throttle, not as a horizontal slider stretched across the entire panel.
    const qreal handle_width = std::min<qreal>(146.0, std::max<qreal>(96.0, outer.width() - 46.0));
    QRectF handle(x - handle_width * 0.5, y - 14.0, handle_width, 28.0);
    QColor handle_fill = accent.darker(250);
    handle_fill.setAlpha(235);
    p.setPen(QPen(accent.lighter(125), 1.4));
    p.setBrush(handle_fill);
    p.drawRoundedRect(handle, 5.0, 5.0);

    p.setPen(QPen(accent.lighter(150), 2.0));
    p.drawLine(QPointF(handle.left() + 8.0, handle.center().y()),
               QPointF(handle.right() - 8.0, handle.center().y()));

    QFont value_font("DejaVu Sans Mono", 12, QFont::Bold);
    p.setFont(value_font);
    p.setPen(text);
    p.drawText(QRectF(outer.left(), 10.0, outer.width(), 22.0),
               Qt::AlignCenter,
               QString("%1 %2").arg(QString::number(value_, 'f', 2), unit_));

    QFont caption_font("DejaVu Sans", 8, QFont::Bold);
    caption_font.setLetterSpacing(QFont::AbsoluteSpacing, 1.1);
    p.setFont(caption_font);
    p.setPen(accent.lighter(135));
    p.drawText(QRectF(outer.left(), height() - 31.0, outer.width(), 18.0),
               Qt::AlignCenter,
               caption_);
}

}  // namespace qube_servo2::gui::ui::widgets
