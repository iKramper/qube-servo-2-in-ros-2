#include "qube_servo2_gui/ui/pages/dashboard_page.hpp"
#include "qube_servo2_gui/ui/widgets/scope_plot.hpp"
#include "qube_servo2_gui/ui/widgets/hud_panel.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <cmath>

namespace qube_servo2::gui::ui::pages {
namespace {
double clean(double v, double eps=1e-6){ return std::abs(v)<eps?0.0:v; }
double wrap(double r){ constexpr double p=3.14159265358979323846,t=2*p; double w=std::fmod(r,t); if(w<0)w+=t; return std::abs(w)<1e-9?0.0:w; }
QString f(double v,int d,double eps=1e-6){ return QString::number(clean(v,eps),'f',d); }
QLabel* makeMetric(const QString& s,QWidget* p){ auto* l=new QLabel(s,p); l->setObjectName("metricValue"); l->setAlignment(Qt::AlignRight|Qt::AlignVCenter); return l; }
QFrame* panel(const QString& title,const QList<QLabel*>& ms,widgets::ScopePlot* plot,QWidget* p){
 auto* q=new widgets::HudPanel(title,p); auto* v=new QVBoxLayout(q); v->setContentsMargins(7,30,7,7); v->setSpacing(3);
 auto* h=new QHBoxLayout(); h->addStretch(1); for(auto* m:ms)h->addWidget(m); v->addLayout(h); v->addWidget(plot,1); return q;
}
}
DashboardPage::DashboardPage(QWidget* parent):QWidget(parent){
 auto* pr=new QVBoxLayout(this); pr->setContentsMargins(0,0,0,0); auto* scroll=new QScrollArea(this); scroll->setWidgetResizable(true); scroll->setFrameShape(QFrame::NoFrame); pr->addWidget(scroll);
 auto* c=new QWidget(scroll); scroll->setWidget(c); auto* root=new QVBoxLayout(c); root->setContentsMargins(10,8,10,8); root->setSpacing(8);
 auto* head=new QHBoxLayout(); auto* title=new QLabel("QUBE-SERVO 2 / LIVE TELEMETRY",this); title->setObjectName("pageTitle"); auto* hint=new QLabel("Hover / zoom tools available on each scope",this); hint->setObjectName("pageSubtitle"); auto* clear=new QPushButton("Clear",this); clear->setObjectName("secondary"); connect(clear,&QPushButton::clicked,this,&DashboardPage::clearCharts); head->addWidget(title); head->addSpacing(14); head->addWidget(hint); head->addStretch(1); head->addWidget(clear); root->addLayout(head);
 auto* g=new QGridLayout(); g->setSpacing(6);
 position_value_=makeMetric("θ -- rad",this); velocity_value_=makeMetric("ω -- rad/s",this); rpm_value_=makeMetric("-- rpm",this); command_value_=makeMetric("Cmd -- V",this); applied_value_=makeMetric("Vm -- V",this); emf_value_=makeMetric("Eb -- V",this); current_value_=makeMetric("i -- A",this); torque_value_=makeMetric("τ -- N·m",this); electrical_power_value_=makeMetric("Pe -- W",this); mechanical_power_value_=makeMetric("Pm -- W",this);
 position_plot_=new widgets::ScopePlot("","rad",{"Position"},this); position_plot_->setMinimumHeight(340); position_plot_->setPiRadiansYAxis(); position_plot_->setWindowSeconds(15);
 speed_plot_=new widgets::ScopePlot("","rad/s",{"Angular velocity"},this); speed_plot_->setMinimumHeight(340); speed_plot_->setInitialYSpan(10); speed_plot_->setWindowSeconds(15);
 electrical_plot_=new widgets::ScopePlot("","V",{"Command","Applied","Back EMF"},this); electrical_plot_->setMinimumHeight(340); electrical_plot_->setYRange(-11,11); electrical_plot_->setSeriesDashed(0,true); electrical_plot_->setWindowSeconds(15);
 current_plot_=new widgets::ScopePlot("","A",{"Current"},this); current_plot_->setMinimumHeight(340); current_plot_->setYRange(-2.2,2.2); current_plot_->setWindowSeconds(15);
 torque_plot_=new widgets::ScopePlot("","N·m",{"Motor torque"},this); torque_plot_->setMinimumHeight(340); torque_plot_->setYRange(-0.1,0.1); torque_plot_->setWindowSeconds(15);
 power_plot_=new widgets::ScopePlot("","W",{"Electrical power","Mechanical power"},this); power_plot_->setMinimumHeight(340); power_plot_->setInitialYSpan(5); power_plot_->setWindowSeconds(15);
 g->addWidget(panel("POSITION [0, 2π)",{position_value_},position_plot_,this),0,0); g->addWidget(panel("SPEED",{velocity_value_,rpm_value_},speed_plot_,this),0,1); g->addWidget(panel("MOTOR VOLTAGES",{command_value_,applied_value_,emf_value_},electrical_plot_,this),1,0); g->addWidget(panel("MOTOR CURRENT",{current_value_},current_plot_,this),1,1); g->addWidget(panel("MOTOR TORQUE",{torque_value_},torque_plot_,this),2,0); g->addWidget(panel("POWER",{electrical_power_value_,mechanical_power_value_},power_plot_,this),2,1); g->setColumnStretch(0,1);g->setColumnStretch(1,1);root->addLayout(g,1);
}
DashboardPage::~DashboardPage(){ shutdown(); }
void DashboardPage::setActive(bool a){ if(shutting_down_)return; for(auto* p:{position_plot_,speed_plot_,electrical_plot_,current_plot_,torque_plot_,power_plot_})p->setActive(a); }
void DashboardPage::shutdown(){ if(shutting_down_)return; shutting_down_=true; for(auto* p:{position_plot_,speed_plot_,electrical_plot_,current_plot_,torque_plot_,power_plot_})p->shutdown(); }
void DashboardPage::updateTelemetry(const models::TelemetrySample& s){ if(shutting_down_||!s.valid)return; const double wr=wrap(s.position_rad); position_value_->setText(QString("θ %1 rad").arg(f(wr,3))); velocity_value_->setText(QString("ω %1 rad/s").arg(f(s.velocity_rad_s,3))); rpm_value_->setText(QString("%1 rpm").arg(f(s.velocity_rpm(),2))); command_value_->setText(QString("Cmd %1 V").arg(f(s.command_voltage,3))); applied_value_->setText(QString("Vm %1 V").arg(f(s.applied_voltage_v,3))); emf_value_->setText(QString("Eb %1 V").arg(f(s.back_emf_v,3))); current_value_->setText(QString("i %1 A").arg(f(s.current_a,4))); torque_value_->setText(QString("τ %1 N·m").arg(f(s.motor_torque_nm,6,1e-8))); electrical_power_value_->setText(QString("Pe %1 W").arg(f(s.electrical_power_w(),5,1e-7))); mechanical_power_value_->setText(QString("Pm %1 W").arg(f(s.mechanical_power_w(),5,1e-7))); if(t0_<0)t0_=s.ros_time_s; const double t=s.ros_time_s-t0_; position_plot_->append(0,t,wr); speed_plot_->append(0,t,clean(s.velocity_rad_s)); electrical_plot_->append(0,t,clean(s.command_voltage)); electrical_plot_->append(1,t,clean(s.applied_voltage_v)); electrical_plot_->append(2,t,clean(s.back_emf_v)); current_plot_->append(0,t,clean(s.current_a)); torque_plot_->append(0,t,clean(s.motor_torque_nm,1e-8)); power_plot_->append(0,t,clean(s.electrical_power_w())); power_plot_->append(1,t,clean(s.mechanical_power_w())); }
void DashboardPage::clearCharts(){ for(auto* p:{position_plot_,speed_plot_,electrical_plot_,current_plot_,torque_plot_,power_plot_})p->clear(); t0_=-1; }
}  // namespace qube_servo2::gui::ui::pages
