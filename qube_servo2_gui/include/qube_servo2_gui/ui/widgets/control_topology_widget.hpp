#pragma once

#include <QString>
#include <QWidget>

class QPaintEvent;
class QPainter;
class QRectF;

namespace qube_servo2::gui::ui::widgets {

class ControlTopologyWidget final : public QWidget {
public:
    explicit ControlTopologyWidget(QWidget* parent = nullptr);
    void setConfiguration(const QString& controller, const QString& mode);
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawArrow_(QPainter& painter, const QPointF& from, const QPointF& to) const;
    void drawBlock_(QPainter& painter, const QRectF& rect, const QString& title,
                    const QString& subtitle = {}) const;
    QString displayController_() const;
    QString equationHtml_() const;

    QString controller_{"motor_voltage_controller"};
    QString mode_{"voltage"};
};

}  // namespace qube_servo2::gui::ui::widgets
