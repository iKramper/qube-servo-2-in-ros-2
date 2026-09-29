#pragma once

#include <QFrame>
#include <QString>

namespace qube_servo2::gui::ui::widgets {

class HudPanel final : public QFrame {
public:
    explicit HudPanel(const QString& title = {}, QWidget* parent = nullptr);

    void setHudTitle(const QString& title);
    QString hudTitle() const;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QString title_;
};

}  // namespace qube_servo2::gui::ui::widgets
