#include "qube_servo2_gui/ui/widgets/scope_plot.hpp"
#include "qube_servo2_gui/ui/style/theme_manager.hpp"

#include <QChart>
#include <QChartView>
#include <QCategoryAxis>
#include <QEvent>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QHBoxLayout>
#include <QLabel>
#include <QLegend>
#include <QLineSeries>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QTimer>
#include <QToolButton>
#include <QToolTip>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

namespace qube_servo2::gui::ui::widgets {

namespace {
constexpr double kMinTimeWindowS = 1.0;
constexpr double kMaxTimeWindowS = 60.0;
constexpr int kHoverPeriodMs = 40;   // ~25 Hz: responsive without burning CPU.
constexpr int kRefreshPeriodMs = 50; // 20 Hz axis / UI refresh.
}

ScopePlot::ScopePlot(const QString& title,
                     const QString& y_label,
                     const QStringList& series_names,
                     QWidget* parent)
    : QFrame(parent) {
    setObjectName("chartFrame");
    setMinimumHeight(190);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(4, 2, 4, 4);
    root->setSpacing(2);

    auto* header = new QHBoxLayout();
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(3);

    title_label_ = new QLabel(title, this);
    title_label_->setObjectName("chartTitle");
    header->addWidget(title_label_);
    header->addStretch(1);

    auto make_button = [this](const QString& text, const QString& tip) {
        auto* b = new QToolButton(this);
        b->setObjectName("chartToolButton");
        b->setText(text);
        b->setToolTip(tip);
        b->setAutoRaise(true);
        b->setFixedHeight(20);
        return b;
    };

    time_in_btn_ = make_button("T−", "Show less time (zoom in). Mouse wheel up does the same.");
    time_out_btn_ = make_button("T+", "Show more time (zoom out). Mouse wheel down does the same.");
    time_label_ = new QLabel(this);
    time_label_->setObjectName("chartWindowLabel");
    time_label_->setAlignment(Qt::AlignCenter);
    time_label_->setMinimumWidth(40);
    y_in_btn_ = make_button("Y−", "Reduce vertical span.");
    y_out_btn_ = make_button("Y+", "Increase vertical span.");
    fit_y_btn_ = make_button("FIT", "Fit visible data once, then keep the Y range locked.");

    header->addWidget(time_in_btn_);
    header->addWidget(time_label_);
    header->addWidget(time_out_btn_);
    header->addSpacing(4);
    header->addWidget(y_in_btn_);
    header->addWidget(y_out_btn_);
    header->addWidget(fit_y_btn_);
    root->addLayout(header);

    chart_ = new QChart();
    chart_->setTitle(QString());
    chart_->legend()->setVisible(series_names.size() > 1);
    chart_->legend()->setAlignment(Qt::AlignTop);
    chart_->setAnimationOptions(QChart::NoAnimation);
    chart_->setMargins(QMargins(0, 0, 0, 0));
    {
        QFont legend_font = chart_->legend()->font();
        const qreal base_pt = legend_font.pointSizeF() > 0.0 ? legend_font.pointSizeF() : 9.0;
        legend_font.setPointSizeF(std::max<qreal>(7.0, base_pt - 1.0));
        chart_->legend()->setFont(legend_font);
    }

    axis_x_ = new QValueAxis(chart_);
    axis_y_ = new QValueAxis(chart_);
    axis_x_->setTitleText("t [s]");
    axis_y_->setTitleText(y_label);
    axis_x_->setLabelFormat("%.1f");
    axis_y_->setLabelFormat("%.3g");
    axis_x_->setTickCount(5);
    axis_y_->setTickCount(5);
    chart_->addAxis(axis_x_, Qt::AlignBottom);
    chart_->addAxis(axis_y_, Qt::AlignLeft);

    for (const auto& name : series_names) {
        auto* s = new QLineSeries(chart_);
        s->setName(name);
        chart_->addSeries(s);
        s->attachAxis(axis_x_);
        s->attachAxis(axis_y_);
        series_.push_back(s);
        series_dashed_.push_back(false);
    }

    view_ = new QChartView(chart_, this);
    view_->setObjectName("scopeChartView");
    view_->setRenderHint(QPainter::Antialiasing, false);
    view_->setContentsMargins(0, 0, 0, 0);
    view_->setRubberBand(QChartView::NoRubberBand);
    view_->setMouseTracking(true);
    view_->viewport()->setMouseTracking(true);
    view_->viewport()->installEventFilter(this);
    root->addWidget(view_, 1);

    connect(time_in_btn_, &QToolButton::clicked, this, [this]() { zoomTime_(0.75); });
    connect(time_out_btn_, &QToolButton::clicked, this, [this]() { zoomTime_(1.333333333); });
    connect(y_in_btn_, &QToolButton::clicked, this, [this]() { scaleY(0.80); });
    connect(y_out_btn_, &QToolButton::clicked, this, [this]() { scaleY(1.25); });
    connect(fit_y_btn_, &QToolButton::clicked, this, &ScopePlot::fitYToVisibleData);

    refresh_timer_ = new QTimer(this);
    refresh_timer_->setInterval(kRefreshPeriodMs);
    refresh_timer_->setTimerType(Qt::CoarseTimer);
    connect(refresh_timer_, &QTimer::timeout, this, &ScopePlot::refreshView_);
    refresh_timer_->start();

    hover_throttle_.start();
    updateTimeLabel_();

    connect(&style::ThemeManager::instance(),
            &style::ThemeManager::themeChanged,
            this,
            [this](style::ThemeId) { applyTheme(); });
    applyTheme();
}

ScopePlot::~ScopePlot() {
    shutdown();
}

void ScopePlot::setActive(bool active) {
    if (shutting_down_ || active_ == active) return;
    active_ = active;

    if (active_) {
        dirty_ = true;
        if (refresh_timer_ && !refresh_timer_->isActive()) refresh_timer_->start();
    } else {
        if (refresh_timer_) refresh_timer_->stop();
        clearCursor_();
    }
}

void ScopePlot::shutdown() {
    if (shutting_down_) return;
    shutting_down_ = true;
    active_ = false;

    if (refresh_timer_) refresh_timer_->stop();

    if (view_ && view_->viewport()) {
        view_->viewport()->removeEventFilter(this);
    }

    QObject::disconnect(
        &style::ThemeManager::instance(),
        nullptr,
        this,
        nullptr);

    QToolTip::hideText();
    hover_active_ = false;

    if (view_ && view_->scene()) {
        auto* scene = view_->scene();
        if (cursor_line_) {
            scene->removeItem(cursor_line_);
            delete cursor_line_;
            cursor_line_ = nullptr;
        }
        for (auto*& marker : cursor_markers_) {
            if (!marker) continue;
            scene->removeItem(marker);
            delete marker;
            marker = nullptr;
        }
    }
    cursor_markers_.clear();

    for (auto* series : series_) {
        if (series) series->clear();
    }

    dirty_ = false;
}

void ScopePlot::append(int series_index, double t_s, double value) {
    if (shutting_down_) return;
    if (series_index < 0 || series_index >= static_cast<int>(series_.size()) ||
        !std::isfinite(t_s) || !std::isfinite(value)) {
        return;
    }

    value = sanitizeValue_(value);

    if (!y_initialized_) {
        const double span = std::max(1e-12, initial_y_span_);
        fixed_y_min_ = value - 0.5 * span;
        fixed_y_max_ = value + 0.5 * span;
        axis_y_->setRange(fixed_y_min_, fixed_y_max_);
        y_initialized_ = true;
    }

    series_[series_index]->append(t_s, value);
    prune_(series_index, t_s);
    dirty_ = true;
}

void ScopePlot::setSeriesData(int series_index, const QList<QPointF>& points) {
    if (shutting_down_ || series_index < 0 || series_index >= static_cast<int>(series_.size())) return;

    QList<QPointF> sanitized;
    sanitized.reserve(points.size());
    for (const auto& p : points) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y())) continue;
        sanitized.append(QPointF(p.x(), sanitizeValue_(p.y())));
    }

    series_[series_index]->replace(sanitized);
    if (!sanitized.isEmpty() && !y_initialized_) {
        const double value = sanitized.constLast().y();
        const double span = std::max(1e-12, initial_y_span_);
        fixed_y_min_ = value - 0.5 * span;
        fixed_y_max_ = value + 0.5 * span;
        axis_y_->setRange(fixed_y_min_, fixed_y_max_);
        y_initialized_ = true;
    }
    dirty_ = true;
}

void ScopePlot::prune_(int series_index, double newest_t) {
    auto* s = series_[series_index];
    int remove_count = 0;
    const double t_min = newest_t - retention_s_;

    while (remove_count < s->count() && s->at(remove_count).x() < t_min) {
        ++remove_count;
    }
    if (s->count() - remove_count > max_points_) {
        remove_count += s->count() - remove_count - max_points_;
    }
    if (remove_count > 0) s->removePoints(0, remove_count);
}

void ScopePlot::refreshView_() {
    if (shutting_down_ || !active_) return;
    if (dirty_) {
        updateAxes_();
        dirty_ = false;
    }
    if (hover_active_) updateCursor_(last_hover_pos_);
}

void ScopePlot::updateAxes_() {
    double latest_t = 0.0;
    bool have_point = false;

    for (auto* s : series_) {
        if (s->count() == 0) continue;
        have_point = true;
        latest_t = std::max(latest_t, s->at(s->count() - 1).x());
    }
    if (!have_point) return;

    if (follow_latest_) {
        const double x_max = std::max(window_s_, latest_t);
        axis_x_->setRange(std::max(0.0, x_max - window_s_), x_max);
    }

    // Deliberately do not auto-rescale Y unless explicitly requested.
    if (auto_range_) fitYToVisibleData();
}

void ScopePlot::clear() {
    for (auto* s : series_) s->clear();
    axis_x_->setRange(0.0, window_s_);
    clearCursor_();
    dirty_ = false;
}

void ScopePlot::setWindowSeconds(double seconds) {
    window_s_ = std::clamp(seconds, kMinTimeWindowS, kMaxTimeWindowS);
    retention_s_ = std::max(retention_s_, window_s_);
    if (!follow_latest_) axis_x_->setRange(0.0, window_s_);
    updateTimeLabel_();
    dirty_ = true;
}

double ScopePlot::windowSeconds() const noexcept { return window_s_; }

void ScopePlot::setRetentionSeconds(double seconds) {
    retention_s_ = std::max(window_s_, seconds);
}

void ScopePlot::setMaxPoints(int max_points) { max_points_ = std::max(100, max_points); }
void ScopePlot::setFollowLatest(bool enabled) { follow_latest_ = enabled; }

void ScopePlot::setSeriesName(int series_index, const QString& name) {
    if (series_index < 0 || series_index >= static_cast<int>(series_.size())) return;
    series_[series_index]->setName(name);
}

void ScopePlot::setSeriesDashed(int series_index, bool dashed) {
    if (series_index < 0 || series_index >= static_cast<int>(series_.size())) return;
    series_dashed_[static_cast<std::size_t>(series_index)] = dashed;
    applyTheme();
}

void ScopePlot::setYLabel(const QString& label) {
    if (axis_y_) axis_y_->setTitleText(label);
}

void ScopePlot::setAutoRange(bool enabled) {
    auto_range_ = enabled;
    if (enabled) dirty_ = true;
}

void ScopePlot::setInitialYSpan(double span) {
    initial_y_span_ = std::max(1e-12, std::abs(span));
}

void ScopePlot::setYRange(double min_value, double max_value) {
    if (min_value > max_value) std::swap(min_value, max_value);
    if (std::abs(max_value - min_value) < 1e-15) return;
    fixed_y_min_ = min_value;
    fixed_y_max_ = max_value;
    y_initialized_ = true;
    auto_range_ = false;
    axis_y_->setRange(min_value, max_value);
}

void ScopePlot::fitYToVisibleData() {
    const double x_min = axis_x_->min();
    const double x_max = axis_x_->max();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    for (auto* s : series_) {
        for (int i = 0; i < s->count(); ++i) {
            const auto p = s->at(i);
            if (p.x() < x_min || p.x() > x_max) continue;
            y_min = std::min(y_min, p.y());
            y_max = std::max(y_max, p.y());
        }
    }

    if (!std::isfinite(y_min) || !std::isfinite(y_max)) return;
    if (std::abs(y_max - y_min) < 1e-12) {
        const double pad = std::max(0.1, std::abs(y_max) * 0.15);
        y_min -= pad;
        y_max += pad;
    } else {
        const double pad = 0.10 * (y_max - y_min);
        y_min -= pad;
        y_max += pad;
    }
    setYRange(y_min, y_max);
}

void ScopePlot::scaleY(double factor) {
    if (factor <= 0.0) return;
    if (!y_initialized_) {
        fitYToVisibleData();
        if (!y_initialized_) return;
    }

    const double center = 0.5 * (axis_y_->min() + axis_y_->max());
    double half = 0.5 * (axis_y_->max() - axis_y_->min()) * factor;

    // Do not allow zooming deeply enough to expose floating-point residue as a signal.
    // This is display-only; raw telemetry / recording is untouched.
    const double min_span = std::max(1e-9, initial_y_span_ * 1e-6);
    half = std::max(half, 0.5 * min_span);
    setYRange(center - half, center + half);
}

void ScopePlot::setPiRadiansYAxis() {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;

    if (!chart_ || !axis_y_) return;

    // Detach and replace the regular numeric axis with a category axis.
    // QCategoryAxis derives from QValueAxis, so the rest of ScopePlot can
    // continue using the same range / mapping logic.
    QValueAxis* old_axis = axis_y_;
    chart_->removeAxis(old_axis);

    auto* pi_axis = new QCategoryAxis(chart_);
    pi_axis->setLabelsPosition(QCategoryAxis::AxisLabelsPositionOnValue);
    pi_axis->setStartValue(0.0);
    pi_axis->append(QString::fromUtf8("π/4"), 0.25 * kPi);
    pi_axis->append(QString::fromUtf8("π/2"), 0.50 * kPi);
    pi_axis->append(QString::fromUtf8("3π/4"), 0.75 * kPi);
    pi_axis->append(QString::fromUtf8("π"), 1.00 * kPi);
    pi_axis->append(QString::fromUtf8("5π/4"), 1.25 * kPi);
    pi_axis->append(QString::fromUtf8("3π/2"), 1.50 * kPi);
    pi_axis->append(QString::fromUtf8("7π/4"), 1.75 * kPi);
    pi_axis->append(QString::fromUtf8("2π"), 2.00 * kPi);
    pi_axis->setRange(0.0, kTwoPi);
    pi_axis->setTitleText(QString::fromUtf8("θ [rad]"));

    axis_y_ = pi_axis;
    chart_->addAxis(axis_y_, Qt::AlignLeft);
    for (auto* series : series_) {
        series->attachAxis(axis_y_);
    }

    delete old_axis;

    fixed_y_min_ = 0.0;
    fixed_y_max_ = kTwoPi;
    y_initialized_ = true;
    auto_range_ = false;

    // A wrapped angular position has a meaningful fixed physical range.
    // Keep time zoom enabled, but hide vertical scaling controls so the
    // π labels always retain their meaning.
    if (y_in_btn_) y_in_btn_->setVisible(false);
    if (y_out_btn_) y_out_btn_->setVisible(false);
    if (fit_y_btn_) fit_y_btn_->setVisible(false);

    applyTheme();
}

void ScopePlot::zoomTime_(double factor) {
    setWindowSeconds(window_s_ * factor);
}

void ScopePlot::updateTimeLabel_() {
    if (time_label_) time_label_->setText(QString::number(window_s_, 'g', 3) + " s");
}

double ScopePlot::zeroThreshold_() const noexcept {
    if (!axis_y_) return 1e-12;
    const double span = std::abs(axis_y_->max() - axis_y_->min());
    // Relative deadband tied to the visible physical range. 1e-8 of full span
    // is far below what can be resolved visually, but suppresses numerical dust.
    return std::max(1e-12, span * 1e-8);
}

double ScopePlot::sanitizeValue_(double value) const noexcept {
    return std::abs(value) < zeroThreshold_() ? 0.0 : value;
}

QString ScopePlot::formatDisplayValue_(double value) const {
    const double v = sanitizeValue_(value);
    if (v == 0.0) return QStringLiteral("0");

    const double span = axis_y_ ? std::abs(axis_y_->max() - axis_y_->min()) : 1.0;
    int decimals = 4;
    if (span >= 1000.0) decimals = 2;
    else if (span >= 100.0) decimals = 3;
    else if (span >= 10.0) decimals = 4;
    else if (span >= 1.0) decimals = 5;
    else if (span >= 0.1) decimals = 6;
    else decimals = 8;

    return QString::number(v, 'f', decimals);
}

std::optional<QPointF> ScopePlot::interpolatedPoint_(QLineSeries* series, double x) const {
    if (!series || series->count() == 0) return std::nullopt;
    if (x < series->at(0).x() || x > series->at(series->count() - 1).x()) return std::nullopt;

    int lo = 0;
    int hi = series->count() - 1;
    while (lo < hi) {
        const int mid = lo + (hi - lo) / 2;
        if (series->at(mid).x() < x) lo = mid + 1;
        else hi = mid;
    }

    if (lo == 0) return series->at(0);
    const QPointF b = series->at(lo);
    const QPointF a = series->at(lo - 1);
    const double dx = b.x() - a.x();
    if (std::abs(dx) < 1e-12) return b;
    const double alpha = std::clamp((x - a.x()) / dx, 0.0, 1.0);
    return QPointF(x, sanitizeValue_(a.y() + alpha * (b.y() - a.y())));
}

bool ScopePlot::eventFilter(QObject* watched, QEvent* event) {
    if (view_ && watched == view_->viewport()) {
        if (event->type() == QEvent::MouseMove) {
            auto* mouse = static_cast<QMouseEvent*>(event);
            last_hover_pos_ = mouse->position().toPoint();
            hover_active_ = true;
            if (hover_throttle_.elapsed() >= kHoverPeriodMs) {
                hover_throttle_.restart();
                updateCursor_(last_hover_pos_);
            }
        } else if (event->type() == QEvent::Leave) {
            hover_active_ = false;
            clearCursor_();
        } else if (event->type() == QEvent::Wheel) {
            auto* wheel = static_cast<QWheelEvent*>(event);
            if (wheel->angleDelta().y() > 0) zoomTime_(0.80);
            else if (wheel->angleDelta().y() < 0) zoomTime_(1.25);
            wheel->accept();
            return true;
        }
    }
    return QFrame::eventFilter(watched, event);
}

void ScopePlot::updateCursor_(const QPoint& viewport_pos) {
    if (!chart_ || !view_ || !view_->scene()) return;

    const QPointF scene_pos = view_->mapToScene(viewport_pos);
    const QPointF chart_pos = chart_->mapFromScene(scene_pos);
    if (!chart_->plotArea().contains(chart_pos)) {
        clearCursor_();
        return;
    }

    const QPointF value = chart_->mapToValue(chart_pos);
    const double x = value.x();
    const auto& spec = style::ThemeManager::instance().currentSpec();

    const QRectF plot = chart_->plotArea();
    const QPointF plot_tl = chart_->mapToScene(plot.topLeft());
    const QPointF plot_br = chart_->mapToScene(plot.bottomRight());
    const QPointF x_scene = chart_->mapToScene(chart_->mapToPosition(QPointF(x, axis_y_->min())));

    if (!cursor_line_) {
        QPen pen(spec.text_muted, 1.0, Qt::DashLine);
        cursor_line_ = view_->scene()->addLine(0, 0, 0, 0, pen);
    }
    cursor_line_->setLine(x_scene.x(), plot_tl.y(), x_scene.x(), plot_br.y());
    cursor_line_->setVisible(true);

    while (cursor_markers_.size() < series_.size()) {
        const std::size_t i = cursor_markers_.size();
        const std::vector<QColor> colors{spec.accent, spec.accent2, spec.ref_color,
                                         QColor("#FFB000"), QColor("#FF6B6B")};
        const QColor c = colors[i % colors.size()];
        cursor_markers_.push_back(view_->scene()->addEllipse(0, 0, 0, 0, QPen(c, 1.5), QBrush(c)));
    }

    QString text = QString("t = %1 s").arg(x, 0, 'f', 3);
    for (std::size_t i = 0; i < series_.size(); ++i) {
        const auto p = interpolatedPoint_(series_[i], x);
        auto* marker = cursor_markers_[i];
        if (!p.has_value()) {
            marker->setVisible(false);
            continue;
        }
        const QPointF ps = chart_->mapToScene(chart_->mapToPosition(*p));
        marker->setRect(ps.x() - 3.5, ps.y() - 3.5, 7.0, 7.0);
        marker->setVisible(true);
        text += QString("\n%1: %2").arg(series_[i]->name(), formatDisplayValue_(p->y()));
    }

    QToolTip::showText(view_->viewport()->mapToGlobal(viewport_pos + QPoint(14, 14)), text, view_);
}

void ScopePlot::clearCursor_() {
    hover_active_ = false;
    if (cursor_line_) cursor_line_->setVisible(false);
    for (auto* marker : cursor_markers_) {
        if (marker) marker->setVisible(false);
    }
    QToolTip::hideText();
}

void ScopePlot::applyTheme() {
    if (shutting_down_) return;
    const auto& spec = style::ThemeManager::instance().currentSpec();
    chart_->setBackgroundBrush(Qt::transparent);
    chart_->setPlotAreaBackgroundBrush(spec.bg);
    chart_->setPlotAreaBackgroundVisible(true);
    chart_->legend()->setLabelColor(spec.text_muted);
    {
        QFont axis_label_font("DejaVu Sans", 10);
        QFont axis_title_font("DejaVu Sans", 11, QFont::DemiBold);
        axis_x_->setLabelsFont(axis_label_font);
        axis_y_->setLabelsFont(axis_label_font);
        axis_x_->setTitleFont(axis_title_font);
        axis_y_->setTitleFont(axis_title_font);
        QFont legend_font = chart_->legend()->font();
        legend_font.setPointSize(10);
        chart_->legend()->setFont(legend_font);
    }
    axis_x_->setLabelsColor(spec.text_muted);
    axis_y_->setLabelsColor(spec.text_muted);
    axis_x_->setTitleBrush(spec.text_muted);
    axis_y_->setTitleBrush(spec.text_muted);

    QColor grid = spec.text_muted;
    grid.setAlpha(75);
    axis_x_->setGridLineColor(grid);
    axis_y_->setGridLineColor(grid);

    const std::vector<QColor> colors{spec.accent, spec.accent2, spec.ref_color,
                                     QColor("#FFB000"), QColor("#FF6B6B")};
    for (std::size_t i = 0; i < series_.size(); ++i) {
        QPen pen(colors[i % colors.size()]);
        pen.setWidthF(1.8);
        pen.setStyle(series_dashed_[i] ? Qt::DashLine : Qt::SolidLine);
        pen.setCapStyle(Qt::RoundCap);
        series_[i]->setPen(pen);
    }

    if (cursor_line_) {
        QColor c = spec.text_muted;
        c.setAlpha(210);
        cursor_line_->setPen(QPen(c, 1.0, Qt::DashLine));
    }
}

}  // namespace qube_servo2::gui::ui::widgets
