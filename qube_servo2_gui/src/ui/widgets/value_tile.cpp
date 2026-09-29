#include "qube_servo2_gui/ui/widgets/value_tile.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QVBoxLayout>
namespace qube_servo2::gui::ui::widgets { ValueTile::ValueTile(const QString&l,const QString&u,QWidget*p):QFrame(p){setObjectName("valueTile");auto*r=new QVBoxLayout(this);r->setContentsMargins(12,10,12,10);label_=new QLabel(l,this);label_->setObjectName("valueTileLabel");value_=new QLabel("--",this);value_->setObjectName("valueTileValue");unit_=new QLabel(u,this);unit_->setObjectName("valueTileUnit");auto*h=new QHBoxLayout();h->addWidget(value_);h->addWidget(unit_);h->addStretch(1);r->addWidget(label_);r->addLayout(h);} void ValueTile::setValue(double v,int d){value_->setText(QLocale::c().toString(v,'f',d));}void ValueTile::setValueText(const QString&t){value_->setText(t);}void ValueTile::setLabel(const QString&t){label_->setText(t);}void ValueTile::setUnit(const QString&t){unit_->setText(t);} }
