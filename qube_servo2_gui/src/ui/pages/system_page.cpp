#include "qube_servo2_gui/ui/pages/system_page.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace qube_servo2::gui::ui::pages {

SystemPage::SystemPage(QWidget* parent)
    : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 10, 12, 10);
    root->setSpacing(8);

    auto* title = new QLabel("QUBE-SERVO 2 / SYSTEM", this);
    title->setObjectName("title");
    root->addWidget(title);

    auto* columns = new QHBoxLayout();
    columns->setSpacing(8);

    auto* ros_panel = new QFrame(this);
    ros_panel->setObjectName("panel");
    auto* ros_grid = new QGridLayout(ros_panel);
    ros_grid->setContentsMargins(12, 10, 12, 10);
    ros_grid->setHorizontalSpacing(12);
    ros_grid->setVerticalSpacing(7);

    auto* ros_title = new QLabel("ROS 2 INTERFACE", ros_panel);
    ros_title->setObjectName("panelTitle");
    ros_grid->addWidget(ros_title, 0, 0, 1, 2);

    connection_label_ = new QLabel("DISCONNECTED", ros_panel);
    connection_label_->setObjectName("metricValue");
    joint_label_ = new QLabel("--", ros_panel);
    command_topic_label_ = new QLabel("--", ros_panel);
    joint_topic_label_ = new QLabel("--", ros_panel);
    dynamic_topic_label_ = new QLabel("--", ros_panel);

    const QList<QLabel*> selectable{joint_label_, command_topic_label_, joint_topic_label_, dynamic_topic_label_};
    for (auto* label : selectable) label->setTextInteractionFlags(Qt::TextSelectableByMouse);

    int row = 1;
    auto add = [&](const QString& name, QLabel* value) {
        auto* key = new QLabel(name, ros_panel);
        key->setObjectName("fieldLabel");
        ros_grid->addWidget(key, row, 0);
        ros_grid->addWidget(value, row, 1);
        ++row;
    };
    add("Connection", connection_label_);
    add("Controlled joint", joint_label_);
    add("Command topic", command_topic_label_);
    add("Joint states", joint_topic_label_);
    add("Dynamic states", dynamic_topic_label_);
    ros_grid->setColumnStretch(1, 1);

    auto* safety_panel = new QFrame(this);
    safety_panel->setObjectName("panel");
    auto* safety_layout = new QVBoxLayout(safety_panel);
    safety_layout->setContentsMargins(12, 10, 12, 10);
    safety_layout->setSpacing(10);

    auto* safety_title = new QLabel("OPERATION & SAFETY", safety_panel);
    safety_title->setObjectName("panelTitle");
    safety_layout->addWidget(safety_title);

    auto* safety_grid = new QGridLayout();
    voltage_limit_label_ = new QLabel("--", safety_panel);
    voltage_limit_label_->setObjectName("metricValue");
    command_label_ = new QLabel("0.000 V", safety_panel);
    command_label_->setObjectName("metricValue");
    safety_grid->addWidget(new QLabel("Voltage limit", safety_panel), 0, 0);
    safety_grid->addWidget(voltage_limit_label_, 0, 1);
    safety_grid->addWidget(new QLabel("Last command", safety_panel), 1, 0);
    safety_grid->addWidget(command_label_, 1, 1);
    safety_grid->setColumnStretch(1, 1);
    safety_layout->addLayout(safety_grid);

    auto* note = new QLabel(
        "The HMI clamps requested voltage to the configured ±V limit. The actuator model keeps the final physical saturation. "
        "Closing the HMI also requests 0 V.", safety_panel);
    note->setObjectName("subtitle");
    note->setWordWrap(true);
    safety_layout->addWidget(note);
    safety_layout->addStretch(1);

    auto* zero_btn = new QPushButton("SEND 0 V NOW", safety_panel);
    zero_btn->setObjectName("danger");
    connect(zero_btn, &QPushButton::clicked, this, &SystemPage::sendZeroRequested);
    safety_layout->addWidget(zero_btn);

    columns->addWidget(ros_panel, 3);
    columns->addWidget(safety_panel, 2);
    columns->setAlignment(Qt::AlignTop);
    root->addLayout(columns);
    root->addStretch(1);
}

void SystemPage::setConfiguration(const QString& joint,
                                  const QString& command_topic,
                                  const QString& joint_topic,
                                  const QString& dynamic_topic,
                                  double voltage_limit) {
    joint_label_->setText(joint);
    command_topic_label_->setText(command_topic);
    joint_topic_label_->setText(joint_topic);
    dynamic_topic_label_->setText(dynamic_topic);
    voltage_limit_label_->setText(QString("±%1 V").arg(voltage_limit, 0, 'f', 3));
}

void SystemPage::setConnected(bool connected) {
    connection_label_->setText(connected ? "LIVE" : "DISCONNECTED / STALE");
}

void SystemPage::setCommand(double voltage) {
    command_label_->setText(QString::number(voltage, 'f', 3) + " V");
}

}  // namespace qube_servo2::gui::ui::pages
