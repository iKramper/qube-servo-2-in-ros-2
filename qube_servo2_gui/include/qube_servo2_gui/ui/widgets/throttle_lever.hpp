#pragma once

#include <QString>
#include <QWidget>

namespace qube_servo2::gui::ui::widgets {

class ThrottleLever final : public QWidget {
    Q_OBJECT

public:
    explicit ThrottleLever(QWidget* parent = nullptr);

    void setRange(double minimum, double maximum);
    void setValue(double value);
    void setUnit(const QString& unit);
    void setCaption(const QString& caption);

    double value() const noexcept;
    double minimum() const noexcept;
    double maximum() const noexcept;

signals:
    void valueChanged(double value);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    double valueFromY_(qreal y) const noexcept;
    qreal yFromValue_(double value) const noexcept;

    double minimum_{-30.0};
    double maximum_{30.0};
    double value_{0.0};
    QString unit_{"rad/s"};
    QString caption_{"VELOCITY COMMAND"};
    bool dragging_{false};
};

}  // namespace qube_servo2::gui::ui::widgets
