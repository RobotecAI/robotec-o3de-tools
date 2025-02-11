
#pragma once

#if !defined(Q_MOC_RUN)
#include "Configs/TerrainShaperConfig.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <QWidget>
#include <QComboBox>
#endif
#include <QButtonGroup>
#include <QHBoxLayout>

Q_DECLARE_METATYPE(AZ::EntityId) // Make Az EntityId available for Qt
Q_DECLARE_METATYPE(TerrainShaper::Config::TerrainShaperActions)
Q_DECLARE_METATYPE(TerrainShaper::Config::TerrainShaperBrushTypes)

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
        QComboBox* m_TerrainDropdown;
        AZStd::vector<AZ::EntityId> m_TerrainEntries;

        QComboBox* m_TerrainActionDropdown;

        Config::TerrainShaperBrushTypes m_SelectedBrush;
        QButtonGroup* m_BrushButtonGroup;
        QHBoxLayout* m_BrushLayout;
    };
} 
