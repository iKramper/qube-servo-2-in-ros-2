#pragma once

#include <QString>
#include <QWidget>

namespace qube_servo2::gui::ui::widgets {

class DialReferenceWidget final : public QWidget {
    Q_OBJECT

public:
    explicit DialReferenceWidget(QWidget* parent = nullptr);

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
    void wheelEvent(QWheelEvent* event) override;

private:
    void setValueFromPoint_(const QPointF& point);
    double normalized_() const noexcept;

    double minimum_{0.0};
    double maximum_{6.283185307179586};
    double value_{0.0};
    QString unit_{"rad"};
    QString caption_{"POSITION SETPOINT"};
};

}  // namespace qube_servo2::gui::ui::widgets
