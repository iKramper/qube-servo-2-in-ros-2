#include "qube_servo2_gui/ui/style/theme_manager.hpp"
#include <QApplication>
#include <QFile>
#include <QPalette>
#include <QTextStream>
namespace qube_servo2::gui::ui::style {
ThemeManager& ThemeManager::instance(){static ThemeManager i;return i;} ThemeManager::ThemeManager(QObject*p):QObject(p),current_spec_(makeSpec_(current_id_)){}
void ThemeManager::apply(ThemeId id){const auto s=makeSpec_(id);if(qApp){qApp->setStyleSheet(loadQss_(s.qss_path));qApp->setPalette(buildPalette_(s));}current_id_=id;current_spec_=s;emit themeChanged(id);} ThemeId ThemeManager::currentId()const noexcept{return current_id_;} const ThemeSpec& ThemeManager::currentSpec()const noexcept{return current_spec_;}
ThemeSpec ThemeManager::makeSpec_(ThemeId id){switch(id){case ThemeId::Light:return{id,"Light",":/theme/light.qss","#3B82F6","#10B981","#8B5CF6","#F8FAFC","#FFFFFF","#0F172A","#64748B"};case ThemeId::Dark:return{id,"Dark",":/theme/dark.qss","#60A5FA","#34D399","#A78BFA","#0A0E1A","#141B2E","#E2E8F0","#94A3B8"};case ThemeId::MATLAB:return{id,"MATLAB",":/theme/matlab.qss","#0072BD","#D95319","#77AC30","#ECECEC","#FAFAFA","#000000","#666666"};case ThemeId::CyberpunkNeon:return{id,"Cyberpunk Neon",":/theme/cyberpunk.qss","#FF2BD6","#00F5FF","#CCFF00","#050510","#0B0B18","#EAF2FF","#8AA0C8"};case ThemeId::TronEvolution:return{id,"Tron Evolution",":/theme/tron_evolution.qss","#00E6FF","#B084FF","#00FF99","#000000","#000F1A","#D0F0FF","#7BB8DD"};default:return{id,"Tron Ares",":/theme/tron_ares.qss","#FF0033","#3399FF","#00E6FF","#000000","#0A0A14","#E0E8F0","#5588BB"};}}
QString ThemeManager::loadQss_(const QString&p){QFile f(p);if(!f.open(QIODevice::ReadOnly|QIODevice::Text))return{};QTextStream s(&f);return s.readAll();}
QPalette ThemeManager::buildPalette_(const ThemeSpec&s){QPalette p;p.setColor(QPalette::Window,s.bg);p.setColor(QPalette::WindowText,s.text);p.setColor(QPalette::Base,s.panel);p.setColor(QPalette::AlternateBase,s.bg.lighter(105));p.setColor(QPalette::Text,s.text);p.setColor(QPalette::PlaceholderText,s.text_muted);p.setColor(QPalette::Button,s.panel);p.setColor(QPalette::ButtonText,s.text);p.setColor(QPalette::Highlight,s.accent);p.setColor(QPalette::HighlightedText,QColor("#FFFFFF"));p.setColor(QPalette::Link,s.accent2);p.setColor(QPalette::ToolTipBase,s.panel);p.setColor(QPalette::ToolTipText,s.text);return p;}
}
