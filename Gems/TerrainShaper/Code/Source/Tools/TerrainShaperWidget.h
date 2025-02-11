
#pragma once

#if !defined(Q_MOC_RUN)
#include "Configs/TerrainShaperConfig.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <QWidget>
#include <QComboBox>
#endif

Q_DECLARE_METATYPE(AZ::EntityId) // Make Az entity available for Qt
Q_DECLARE_METATYPE(TerrainShaper::Config::TerrainShaperActions) // Make Az entity available for Qt

namespace TerrainShaper
{
    class TerrainShaperWidget
        : public QWidget
    {
        Q_OBJECT
    public:
        explicit TerrainShaperWidget(QWidget* parent = nullptr);

    private slots:
        void OnTerrainRefreshButtonClicked();   //!< Refresh Current Terrain List based on selected entities in editor.
        void OnTerrainDropdownChanged(int index);  // Handle selection change
        void OnTerrainActionDropdownChanged(int index);  // Handle selection change

    private:
        QComboBox* m_TerrainDropdown;
        AZStd::vector<AZ::EntityId> m_TerrainEntries;

        QComboBox* m_TerrainActionDropdown;
    };
} 
