#include "qube_servo2_gui/ui/style/theme_manager.hpp"

#include <QApplication>
#include <QFile>
#include <QPalette>
#include <QTextStream>

namespace qube_servo2::gui::ui::style {

ThemeManager& ThemeManager::instance() {
    static ThemeManager instance;
    return instance;
}

ThemeManager::ThemeManager(QObject* parent)
    : QObject(parent), current_spec_(makeSpec_(current_id_)) {}

void ThemeManager::apply(ThemeId id) {
    const ThemeSpec spec = makeSpec_(id);
    if (qApp) {
        qApp->setStyleSheet(loadQss_(spec.qss_path));
        qApp->setPalette(buildPalette_(spec));
    }
    current_id_ = id;
    current_spec_ = spec;
    emit themeChanged(id);
}

ThemeId ThemeManager::currentId() const noexcept { return current_id_; }
const ThemeSpec& ThemeManager::currentSpec() const noexcept { return current_spec_; }

ThemeSpec ThemeManager::makeSpec_(ThemeId id) {
    switch (id) {
    case ThemeId::Light:
        return {id, "Light", ":/theme/light.qss", "#3B82F6", "#10B981", "#8B5CF6",
                "#F8FAFC", "#FFFFFF", "#0F172A", "#64748B"};
    case ThemeId::Dark:
        return {id, "Dark", ":/theme/dark.qss", "#60A5FA", "#34D399", "#A78BFA",
                "#0A0E1A", "#141B2E", "#E2E8F0", "#94A3B8"};
    case ThemeId::MATLAB:
        return {id, "MATLAB", ":/theme/matlab.qss", "#0072BD", "#D95319", "#77AC30",
                "#ECECEC", "#FAFAFA", "#000000", "#666666"};
    case ThemeId::CyberpunkNeon:
        return {id, "Cyberpunk Neon", ":/theme/cyberpunk.qss", "#FF2BD6", "#00F5FF", "#CCFF00",
                "#050510", "#0B0B18", "#EAF2FF", "#8AA0C8"};
    case ThemeId::TronEvolution:
        return {id, "Tron Evolution", ":/theme/tron_evolution.qss", "#00E6FF", "#B084FF", "#00FF99",
                "#000000", "#000F1A", "#D0F0FF", "#7BB8DD"};
    case ThemeId::TronAres:
    default:
        return {id, "Tron Ares", ":/theme/tron_ares.qss", "#FF0033", "#3399FF", "#00E6FF",
                "#000000", "#0A0A14", "#E0E8F0", "#5588BB"};
    }
}

QString ThemeManager::loadQss_(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return {};
    QTextStream stream(&file);
    return stream.readAll();
}

QPalette ThemeManager::buildPalette_(const ThemeSpec& spec) {
    QPalette palette;
    palette.setColor(QPalette::Window, spec.bg);
    palette.setColor(QPalette::WindowText, spec.text);
    palette.setColor(QPalette::Base, spec.panel);
    palette.setColor(QPalette::AlternateBase, spec.bg.lighter(105));
    palette.setColor(QPalette::Text, spec.text);
    palette.setColor(QPalette::Button, spec.panel);
    palette.setColor(QPalette::ButtonText, spec.text);
    palette.setColor(QPalette::Highlight, spec.accent);
    palette.setColor(QPalette::HighlightedText, QColor("#FFFFFF"));
    palette.setColor(QPalette::Link, spec.accent2);
    palette.setColor(QPalette::ToolTipBase, spec.panel);
    palette.setColor(QPalette::ToolTipText, spec.text);
    return palette;
}

}  // namespace qube_servo2::gui::ui::style
