#include "qube_servo2_gui/ui/widgets/dial_reference_widget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::ui::widgets {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kStartAngleDeg = 225.0;
constexpr double kSweepDeg = 270.0;
}

DialReferenceWidget::DialReferenceWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(210, 220);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    setCursor(Qt::PointingHandCursor);
    setFocusPolicy(Qt::StrongFocus);
}

void DialReferenceWidget::setRange(double minimum, double maximum) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum) return;
    minimum_ = minimum;
    maximum_ = maximum;
    setValue(value_);
    update();
}

void DialReferenceWidget::setValue(double value) {
    if (!std::isfinite(value)) return;
    const double clamped = std::clamp(value, minimum_, maximum_);
    if (std::abs(clamped - value_) < 1e-10) return;
    value_ = clamped;
    update();
    emit valueChanged(value_);
}

void DialReferenceWidget::setUnit(const QString& unit) {
    unit_ = unit;
    update();
}

void DialReferenceWidget::setCaption(const QString& caption) {
    caption_ = caption;
    update();
}

double DialReferenceWidget::value() const noexcept { return value_; }
double DialReferenceWidget::minimum() const noexcept { return minimum_; }
double DialReferenceWidget::maximum() const noexcept { return maximum_; }

double DialReferenceWidget::normalized_() const noexcept {
    const double span = maximum_ - minimum_;
    if (span <= 0.0) return 0.0;
    return std::clamp((value_ - minimum_) / span, 0.0, 1.0);
}

void DialReferenceWidget::setValueFromPoint_(const QPointF& point) {
    const QPointF c(width() * 0.5, height() * 0.46);
    const double dx = point.x() - c.x();
    const double dy = point.y() - c.y();
    if ((dx * dx + dy * dy) < 36.0) return;

    double angle = std::atan2(-dy, dx) * 180.0 / kPi;
    if (angle < 0.0) angle += 360.0;

    double clockwise = std::fmod(kStartAngleDeg - angle + 360.0, 360.0);
    if (clockwise > kSweepDeg) {
        const double to_start = std::min(std::abs(angle - kStartAngleDeg), 360.0 - std::abs(angle - kStartAngleDeg));
        const double end_angle = std::fmod(kStartAngleDeg - kSweepDeg + 360.0, 360.0);
        const double to_end = std::min(std::abs(angle - end_angle), 360.0 - std::abs(angle - end_angle));
        clockwise = (to_start < to_end) ? 0.0 : kSweepDeg;
    }

    const double alpha = std::clamp(clockwise / kSweepDeg, 0.0, 1.0);
    setValue(minimum_ + alpha * (maximum_ - minimum_));
}

void DialReferenceWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        setValueFromPoint_(event->position());
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void DialReferenceWidget::mouseMoveEvent(QMouseEvent* event) {
    if (event->buttons() & Qt::LeftButton) {
        setValueFromPoint_(event->position());
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void DialReferenceWidget::wheelEvent(QWheelEvent* event) {
    const double step = (maximum_ - minimum_) / 180.0;
    if (event->angleDelta().y() > 0) setValue(value_ + step);
    else if (event->angleDelta().y() < 0) setValue(value_ - step);
    event->accept();
}

void DialReferenceWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    const auto pal = palette();
    QColor accent = pal.color(QPalette::Highlight);
    QColor text = pal.color(QPalette::WindowText);
    QColor muted = pal.color(QPalette::PlaceholderText);
    if (!muted.isValid()) muted = text.darker(170);

    const qreal diameter = std::min(width() - 44.0, height() - 72.0);
    const QPointF center(width() * 0.5, height() * 0.45);
    const QRectF arc(center.x() - diameter * 0.5,
                     center.y() - diameter * 0.5,
                     diameter,
                     diameter);

    QColor track = muted;
    track.setAlpha(75);
    p.setPen(QPen(track, 8.0, Qt::SolidLine, Qt::RoundCap));
    p.drawArc(arc, static_cast<int>(kStartAngleDeg * 16.0), static_cast<int>(-kSweepDeg * 16.0));

    QColor active = accent;
    active.setAlpha(230);
    p.setPen(QPen(active, 8.0, Qt::SolidLine, Qt::RoundCap));
    p.drawArc(arc,
              static_cast<int>(kStartAngleDeg * 16.0),
              static_cast<int>(-kSweepDeg * normalized_() * 16.0));

    p.setPen(QPen(muted, 1.0));
    for (int i = 0; i <= 12; ++i) {
        const double alpha = static_cast<double>(i) / 12.0;
        const double deg = kStartAngleDeg - kSweepDeg * alpha;
        const double rad = deg * kPi / 180.0;
        const double r0 = diameter * 0.5 + 8.0;
        const double r1 = r0 + (i % 3 == 0 ? 10.0 : 6.0);
        const QPointF a(center.x() + std::cos(rad) * r0,
                        center.y() - std::sin(rad) * r0);
        const QPointF b(center.x() + std::cos(rad) * r1,
                        center.y() - std::sin(rad) * r1);
        p.drawLine(a, b);
    }

    QColor hub = pal.color(QPalette::Base);
    hub = hub.lighter(118);
    p.setPen(QPen(accent.darker(115), 1.4));
    p.setBrush(hub);
    p.drawEllipse(center, diameter * 0.31, diameter * 0.31);

    const double pointer_deg = kStartAngleDeg - kSweepDeg * normalized_();
    const double pointer_rad = pointer_deg * kPi / 180.0;
    p.setPen(QPen(accent.lighter(125), 4.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(center,
               QPointF(center.x() + std::cos(pointer_rad) * diameter * 0.25,
                       center.y() - std::sin(pointer_rad) * diameter * 0.25));

    QFont value_font("DejaVu Sans Mono");
    value_font.setBold(true);
    value_font.setPointSizeF(16.0);
    p.setFont(value_font);
    p.setPen(text);
    p.drawText(QRectF(8.0, center.y() - 16.0, width() - 16.0, 32.0),
               Qt::AlignCenter,
               QString::number(value_, 'f', 3));

    QFont unit_font = value_font;
    unit_font.setPointSizeF(8.5);
    unit_font.setBold(false);
    p.setFont(unit_font);
    p.setPen(muted);
    p.drawText(QRectF(8.0, center.y() + 17.0, width() - 16.0, 18.0),
               Qt::AlignCenter,
               unit_);

    QFont caption_font = unit_font;
    caption_font.setBold(true);
    caption_font.setPointSizeF(8.0);
    caption_font.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
    p.setFont(caption_font);
    p.setPen(accent.lighter(135));
    p.drawText(QRectF(8.0, height() - 30.0, width() - 16.0, 20.0),
               Qt::AlignCenter,
               caption_);
}

}  // namespace qube_servo2::gui::ui::widgets
