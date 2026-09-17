#pragma once

#include <QWidget>

class QLabel;

namespace qube_servo2::gui::ui::pages {

class SystemPage final : public QWidget {
    Q_OBJECT

public:
    explicit SystemPage(QWidget* parent = nullptr);

    void setConfiguration(const QString& joint,
                          const QString& command_topic,
                          const QString& joint_topic,
                          const QString& dynamic_topic,
                          double voltage_limit);

public slots:
    void setConnected(bool connected);
    void setCommand(double voltage);

signals:
    void sendZeroRequested();

private:
    QLabel* connection_label_{nullptr};
    QLabel* joint_label_{nullptr};
    QLabel* command_topic_label_{nullptr};
    QLabel* joint_topic_label_{nullptr};
    QLabel* dynamic_topic_label_{nullptr};
    QLabel* voltage_limit_label_{nullptr};
    QLabel* command_label_{nullptr};
};

}  // namespace qube_servo2::gui::ui::pages
