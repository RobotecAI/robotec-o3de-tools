
#pragma once

#if !defined(Q_MOC_RUN)
#include "Configs/TerrainShaperConfig.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <QWidget>
#include <QComboBox>
#endif
#include <QButtonGroup>
#include <QHBoxLayout>
#include <QIcon>
#include <QPixmap>
#include <QSize>

Q_DECLARE_METATYPE(AZ::EntityId) // Make Az EntityId available for Qt
Q_DECLARE_METATYPE(TerrainShaper::Config::ShaperActions)
Q_DECLARE_METATYPE(TerrainShaper::Config::BrushTypes)
Q_DECLARE_METATYPE(TerrainShaper::Config::BrushInfo)
Q_DECLARE_METATYPE(TerrainShaper::Config::TerrainSelectSettings)

namespace TerrainShaper
{
    class TerrainShaperWidget
        : public QWidget
    {
        Q_OBJECT
    public:
        explicit TerrainShaperWidget(QWidget* parent = nullptr);
        void CreateBrushSelect();

    private slots:
        void OnTerrainRefreshButtonClicked();   //!< Refresh Current Terrain List based on selected entities in editor.
        void OnTerrainDropdownChanged(int index);  // Handle selection change
        void OnTerrainActionDropdownChanged(int index);  // Handle selection change
        void OnBrushSelected(int index);

    private:
        // Terrain Selection
        QComboBox* m_TerrainDropdown;
        AZStd::vector<AZ::EntityId> m_TerrainEntries;
        Config::TerrainSelectSettings m_TerrainSelectSettings;

        // Terrain Brush Actions
        QComboBox* m_TerrainActionDropdown;

        // Terrain Brush
        Config::BrushTypes m_SelectedBrush;
        QButtonGroup* m_BrushButtonGroup;
        QHBoxLayout* m_BrushLayout;
    };
} 
