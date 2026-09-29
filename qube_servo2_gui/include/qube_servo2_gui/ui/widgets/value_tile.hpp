#pragma once

#include <QFrame>
#include <QString>

class QLabel;

namespace qube_servo2::gui::ui::widgets {

class ValueTile final : public QFrame {
    Q_OBJECT

public:
    ValueTile(
        const QString& label,
        const QString& unit,
        QWidget* parent = nullptr);

    void setValue(double value, int decimals = 3);
    void setValueText(const QString& text);
    void setLabel(const QString& label);
    void setUnit(const QString& unit);

private:
    QLabel* label_{nullptr};
    QLabel* value_{nullptr};
    QLabel* unit_{nullptr};
};

}  // namespace qube_servo2::gui::ui::widgets
