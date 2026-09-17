#pragma once

#include <QColor>
#include <QObject>
#include <QString>

class QPalette;

namespace qube_servo2::gui::ui::style {

enum class ThemeId {
    Light = 0,
    Dark,
    MATLAB,
    CyberpunkNeon,
    TronAres,
    TronEvolution
};

struct ThemeSpec {
    ThemeId id{ThemeId::TronAres};
    QString display_name;
    QString qss_path;
    QColor accent;
    QColor accent2;
    QColor ref_color;
    QColor bg;
    QColor panel;
    QColor text;
    QColor text_muted;
};

class ThemeManager final : public QObject {
    Q_OBJECT

public:
    static ThemeManager& instance();

    void apply(ThemeId id);
    ThemeId currentId() const noexcept;
    const ThemeSpec& currentSpec() const noexcept;

signals:
    void themeChanged(ThemeId id);

private:
    explicit ThemeManager(QObject* parent = nullptr);

    static ThemeSpec makeSpec_(ThemeId id);
    static QString loadQss_(const QString& path);
    static QPalette buildPalette_(const ThemeSpec& spec);

    ThemeId current_id_{ThemeId::TronAres};
    ThemeSpec current_spec_{};
};

}  // namespace qube_servo2::gui::ui::style
