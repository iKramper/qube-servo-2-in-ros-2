#pragma once

#include <QElapsedTimer>
#include <QEvent>
#include <QFrame>
#include <QList>
#include <QPointF>
#include <QStringList>

#include <optional>
#include <vector>

class QChart;
class QChartView;
class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QLabel;
class QLineSeries;
class QTimer;
class QToolButton;
class QValueAxis;

namespace qube_servo2::gui::ui::widgets {

class ScopePlot final : public QFrame {
    Q_OBJECT

public:
    ScopePlot(const QString& title,
              const QString& y_label,
              const QStringList& series_names,
              QWidget* parent = nullptr);
    ~ScopePlot() override;

    void append(int series_index, double t_s, double value);
    void setSeriesData(int series_index, const QList<QPointF>& points);
    void clear();

    // Time zoom only. The selected window remains fixed while live data arrives.
    void setWindowSeconds(double seconds);
    double windowSeconds() const noexcept;
    void setRetentionSeconds(double seconds);
    void setMaxPoints(int max_points);
    void setFollowLatest(bool enabled);
    void setActive(bool active);
    void shutdown();

    // Series presentation. Reference / command signals can be shown dashed.
    void setSeriesName(int series_index, const QString& name);
    void setSeriesDashed(int series_index, bool dashed);
    void setYLabel(const QString& label);

    // Y is manual by default. FIT Y is a one-shot operation and then remains locked.
    void setAutoRange(bool enabled);
    void setYRange(double min_value, double max_value);
    void setInitialYSpan(double span);
    void fitYToVisibleData();
    void scaleY(double factor);

    // Replace the normal numeric Y axis with a fixed [0, 2π] position axis
    // labeled in π/4 increments. Intended for wrapped rotary position only.
    void setPiRadiansYAxis();

public slots:
    void applyTheme();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
    void refreshView_();

private:
    void updateAxes_();
    void prune_(int series_index, double newest_t);
    void zoomTime_(double factor);
    void updateTimeLabel_();
    double zeroThreshold_() const noexcept;
    double sanitizeValue_(double value) const noexcept;
    QString formatDisplayValue_(double value) const;

    std::optional<QPointF> interpolatedPoint_(QLineSeries* series, double x) const;
    void updateCursor_(const QPoint& viewport_pos);
    void clearCursor_();

    QChart* chart_{nullptr};
    QChartView* view_{nullptr};
    QValueAxis* axis_x_{nullptr};
    QValueAxis* axis_y_{nullptr};
    std::vector<QLineSeries*> series_;
    std::vector<bool> series_dashed_;

    QLabel* title_label_{nullptr};
    QLabel* time_label_{nullptr};
    QToolButton* time_in_btn_{nullptr};
    QToolButton* time_out_btn_{nullptr};
    QToolButton* y_in_btn_{nullptr};
    QToolButton* y_out_btn_{nullptr};
    QToolButton* fit_y_btn_{nullptr};

    QTimer* refresh_timer_{nullptr};
    bool dirty_{false};

    double window_s_{15.0};
    double retention_s_{60.0};
    int max_points_{3000};
    bool follow_latest_{true};
    bool active_{true};
    bool shutting_down_{false};

    bool auto_range_{false};
    bool y_initialized_{false};
    double initial_y_span_{2.0};
    double fixed_y_min_{-1.0};
    double fixed_y_max_{1.0};

    QElapsedTimer hover_throttle_;
    QPoint last_hover_pos_{};
    bool hover_active_{false};
    QGraphicsLineItem* cursor_line_{nullptr};
    std::vector<QGraphicsEllipseItem*> cursor_markers_;
};

}  // namespace qube_servo2::gui::ui::widgets
