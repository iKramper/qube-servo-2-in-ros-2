#include "qube_servo2_gui/ui/widgets/value_tile.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QVBoxLayout>

namespace qube_servo2::gui::ui::widgets {

ValueTile::ValueTile(const QString& label, const QString& unit, QWidget* parent)
    : QFrame(parent) {
    setObjectName("valueTile");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 10, 12, 10);
    root->setSpacing(5);

    label_ = new QLabel(label, this);
    label_->setObjectName("valueTileLabel");

    value_ = new QLabel("--", this);
    value_->setObjectName("valueTileValue");

    unit_ = new QLabel(unit, this);
    unit_->setObjectName("valueTileUnit");

    auto* value_row = new QHBoxLayout();
    value_row->setContentsMargins(0, 0, 0, 0);
    value_row->setSpacing(8);
    value_row->addWidget(value_);
    value_row->addWidget(unit_);
    value_row->addStretch(1);

    root->addWidget(label_);
    root->addLayout(value_row);
}

void ValueTile::setValue(double value, int decimals) {
    value_->setText(QLocale::c().toString(value, 'f', decimals));
}

void ValueTile::setValueText(const QString& text) { value_->setText(text); }
void ValueTile::setLabel(const QString& label) { label_->setText(label); }
void ValueTile::setUnit(const QString& unit) { unit_->setText(unit); }

}  // namespace qube_servo2::gui::ui::widgets
