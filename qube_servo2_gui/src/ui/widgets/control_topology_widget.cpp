#include "qube_servo2_gui/ui/widgets/control_topology_widget.hpp"

#include <QLineF>
#include <QPainter>
#include <QPalette>
#include <QPolygonF>
#include <QTextDocument>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::ui::widgets {
namespace { constexpr double kPi = 3.14159265358979323846; }

ControlTopologyWidget::ControlTopologyWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(185);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
}

void ControlTopologyWidget::setConfiguration(const QString& controller, const QString& mode) {
    if (controller_ == controller && mode_ == mode) return;
    controller_ = controller;
    mode_ = mode;
    update();
}

QSize ControlTopologyWidget::minimumSizeHint() const { return {520, 185}; }

QString ControlTopologyWidget::displayController_() const {
    if (controller_ == "qube_pid_controller") return "PID";
    if (controller_ == "qube_state_feedback_controller") return "STATE FEEDBACK";
    return "OPEN-LOOP VOLTAGE";
}

QString ControlTopologyWidget::equationHtml_() const {
    const QString p = "<div style='font-size:13px'><b>";
    if (controller_ == "qube_pid_controller") {
        if (mode_ == "velocity") {
            return p + "V[k] = K<sub>p</sub>e<sub>&omega;</sub>[k] + K<sub>i</sub>I<sub>&omega;</sub>[k]"
                "</b><br/><span style='font-size:11px'>e<sub>&omega;</sub>[k] = &omega;<sub>r</sub>[k] - &omega;[k]</span></div>";
        }
        return p + "V[k] = K<sub>p</sub>e<sub>&theta;</sub>[k] + K<sub>i</sub>I<sub>&theta;</sub>[k] - K<sub>d</sub>&omega;[k]"
            "</b><br/><span style='font-size:11px'>e<sub>&theta;</sub>[k] = &theta;<sub>r</sub>[k] - &theta;[k]</span></div>";
    }
    if (controller_ == "qube_state_feedback_controller") {
        if (mode_ == "velocity") {
            return p + "&theta;<sub>r</sub>[k] = &theta;<sub>r</sub>[k-1] + T<sub>s</sub>&omega;<sub>r</sub>[k]"
                "</b><br/><span style='font-size:12px'>V[k] = k<sub>&theta;</sub>(&theta;<sub>r</sub>-&theta;) + k<sub>&omega;</sub>(&omega;<sub>r</sub>-&omega;)</span></div>";
        }
        return p + "V[k] = k<sub>&theta;</sub>(&theta;<sub>r</sub>[k]-&theta;[k]) - k<sub>&omega;</sub>&omega;[k]"
            "</b><br/><span style='font-size:11px'>x = [&theta;&nbsp;&nbsp;&omega;]<sup>T</sup>, x<sub>r</sub> = [&theta;<sub>r</sub>&nbsp;&nbsp;0]<sup>T</sup></span></div>";
    }
    return p + "V[k] = r<sub>V</sub>[k]</b><br/><span style='font-size:11px'>Direct voltage excitation, without feedback.</span></div>";
}

void ControlTopologyWidget::drawBlock_(QPainter& painter, const QRectF& rect,
                                       const QString& title, const QString& subtitle) const {
    QColor fill = palette().color(QPalette::AlternateBase);
    QColor border = palette().color(QPalette::Highlight);
    QColor text = palette().color(QPalette::WindowText);
    fill.setAlpha(210); border.setAlpha(220);
    painter.setPen(QPen(border, 1.5)); painter.setBrush(fill);
    painter.drawRoundedRect(rect, 7, 7);
    painter.setPen(text);
    QFont f = painter.font(); f.setBold(true); painter.setFont(f);
    QRectF a = rect.adjusted(4, 4, -4, subtitle.isEmpty() ? -4 : -rect.height()*0.42);
    painter.drawText(a, Qt::AlignCenter, title);
    if (!subtitle.isEmpty()) {
        f.setBold(false); f.setPointSizeF(std::max(7.0, f.pointSizeF()-1.2)); painter.setFont(f);
        QColor muted = palette().color(QPalette::PlaceholderText); if (!muted.isValid()) muted = text;
        painter.setPen(muted);
        painter.drawText(rect.adjusted(4, rect.height()*0.52, -4, -3), Qt::AlignHCenter|Qt::AlignTop, subtitle);
    }
}

void ControlTopologyWidget::drawArrow_(QPainter& painter, const QPointF& from, const QPointF& to) const {
    QColor c = palette().color(QPalette::Highlight);
    painter.setPen(QPen(c, 1.7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawLine(from, to);
    QLineF line(from, to); if (line.length() < 1.0) return;
    const double angle = std::atan2(-line.dy(), line.dx());
    constexpr double s = 7.0;
    QPointF p1 = to - QPointF(std::cos(angle+kPi/6)*s, -std::sin(angle+kPi/6)*s);
    QPointF p2 = to - QPointF(std::cos(angle-kPi/6)*s, -std::sin(angle-kPi/6)*s);
    painter.setBrush(c); painter.drawPolygon(QPolygonF{to,p1,p2});
}

void ControlTopologyWidget::paintEvent(QPaintEvent* event) {
    QWidget::paintEvent(event);
    QPainter painter(this); painter.setRenderHint(QPainter::Antialiasing); painter.setRenderHint(QPainter::TextAntialiasing);
    const QRectF area = rect().adjusted(8,5,-8,-5); if (area.width()<300 || area.height()<120) return;
    QColor text = palette().color(QPalette::WindowText); QColor muted = palette().color(QPalette::PlaceholderText);
    if (!muted.isValid()) muted=text;
    QFont hf=painter.font(); hf.setBold(true); painter.setFont(hf); painter.setPen(text);
    const QString mode_label = mode_=="position"?"POSITION":mode_=="velocity"?"VELOCITY":"VOLTAGE";
    painter.drawText(QRectF(area.left(),area.top(),area.width(),20),Qt::AlignLeft|Qt::AlignVCenter,displayController_()+"  /  "+mode_label);
    const qreal cy=area.top()+66; const qreal l=area.left()+8; const qreal r=area.right()-8;

    if (controller_=="motor_voltage_controller") {
        QRectF b1(l,cy-22,70,44), b2(l+102,cy-26,108,52), b3(l+246,cy-26,120,52);
        drawBlock_(painter,b1,"rV","[V]"); drawBlock_(painter,b2,"Voltage","V = rV"); drawBlock_(painter,b3,"QUBE-Servo 2","motor + mechanics");
        drawArrow_(painter,{b1.right(),cy},{b2.left(),cy}); drawArrow_(painter,{b2.right(),cy},{b3.left(),cy}); drawArrow_(painter,{b3.right(),cy},{r,cy});
    } else if (controller_=="qube_pid_controller") {
        const QString ref=mode_=="velocity"?"ωr":"θr"; const QString fb=mode_=="velocity"?"ω":"θ, ω";
        QRectF b1(l,cy-22,62,44), sum(l+86,cy-20,40,40), ctrl(l+154,cy-26,98,52), plant(l+286,cy-26,120,52);
        drawBlock_(painter,b1,ref,mode_=="velocity"?"[rad/s]":"[rad]"); drawBlock_(painter,sum,"Σ","+ / −");
        drawBlock_(painter,ctrl,mode_=="velocity"?"PI":"PID","digital"); drawBlock_(painter,plant,"QUBE-Servo 2","V → response");
        drawArrow_(painter,{b1.right(),cy},{sum.left(),cy}); drawArrow_(painter,{sum.right(),cy},{ctrl.left(),cy}); drawArrow_(painter,{ctrl.right(),cy},{plant.left(),cy}); drawArrow_(painter,{plant.right(),cy},{r,cy});
        const qreal fy=cy+42; painter.setPen(QPen(palette().color(QPalette::Highlight),1.2)); painter.drawLine(QPointF(plant.right()+5,cy),QPointF(plant.right()+5,fy)); painter.drawLine(QPointF(plant.right()+5,fy),QPointF(sum.center().x(),fy)); drawArrow_(painter,{sum.center().x(),fy},{sum.center().x(),sum.bottom()});
        painter.setPen(muted); painter.drawText(QRectF(sum.center().x()+8,fy-17,100,16),Qt::AlignLeft|Qt::AlignVCenter,fb+" feedback");
    } else {
        const QString ref=mode_=="velocity"?"ωr":"θr";
        QRectF b1(l,cy-22,62,44), gen(l+82,cy-26,110,52), gain(l+220,cy-26,100,52), plant(l+350,cy-26,120,52);
        drawBlock_(painter,b1,ref,mode_=="velocity"?"[rad/s]":"[rad]"); drawBlock_(painter,gen,"xᵣ generator",mode_=="velocity"?"θᵣ = ∫ωᵣdt":"xᵣ=[θᵣ 0]ᵀ");
        drawBlock_(painter,gain,"K(xᵣ − x)","state feedback"); drawBlock_(painter,plant,"QUBE-Servo 2","V → x");
        drawArrow_(painter,{b1.right(),cy},{gen.left(),cy}); drawArrow_(painter,{gen.right(),cy},{gain.left(),cy}); drawArrow_(painter,{gain.right(),cy},{plant.left(),cy}); drawArrow_(painter,{plant.right(),cy},{r,cy});
        const qreal fy=cy+42; painter.setPen(QPen(palette().color(QPalette::Highlight),1.2)); painter.drawLine(QPointF(plant.right()+5,cy),QPointF(plant.right()+5,fy)); painter.drawLine(QPointF(plant.right()+5,fy),QPointF(gain.center().x(),fy)); drawArrow_(painter,{gain.center().x(),fy},{gain.center().x(),gain.bottom()});
        painter.setPen(muted); painter.drawText(QRectF(gain.center().x()+8,fy-17,100,16),Qt::AlignLeft|Qt::AlignVCenter,"x=[θ ω]ᵀ");
    }

    QTextDocument eq; eq.setDocumentMargin(0); eq.setDefaultFont(font()); eq.setHtml(equationHtml_()); eq.setTextWidth(area.width());
    const qreal y=area.bottom()-50; painter.save(); painter.translate(area.left(),y); eq.drawContents(&painter,QRectF(0,0,area.width(),48)); painter.restore();
}

}  // namespace qube_servo2::gui::ui::widgets
