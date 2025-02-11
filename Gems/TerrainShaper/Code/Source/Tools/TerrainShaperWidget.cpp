
#include "TerrainShaperWidget.h"

#include <Utils/TerrainShaperUtils.h>

#include <AzCore/Utils/Utils.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>

namespace TerrainShaper
{
    TerrainShaperWidget::TerrainShaperWidget(QWidget* parent)
        : QWidget(parent)
    {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);

        QLabel* introLabel = new QLabel(QObject::tr("Terrain Shaping Tool"), this);
        mainLayout->addWidget(introLabel, 0, Qt::AlignTop);

        // Get Terrainn List
        QPushButton* terrainButton = new QPushButton(QObject::tr("Refresh Available Terrains"), this);
        mainLayout->addWidget(terrainButton, 0, Qt::AlignCenter);

        connect(terrainButton, &QPushButton::clicked, this, &TerrainShaperWidget::OnTerrainRefreshButtonClicked);

        // Add Available Terrains to ComboBox
        m_TerrainDropdown = new QComboBox(this);
        mainLayout->addWidget(m_TerrainDropdown, 0, Qt::AlignCenter);

        connect(m_TerrainDropdown, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &TerrainShaperWidget::OnTerrainDropdownChanged);

        // List Actions
        m_TerrainActionDropdown = new QComboBox(this);
        m_TerrainActionDropdown->addItem("Flatten Terrain");
        m_TerrainActionDropdown->addItem("Raise Terrain");
        m_TerrainActionDropdown->addItem("Lower Terrain");
        m_TerrainActionDropdown->addItem("Smooth Terrain");

        mainLayout->addWidget(m_TerrainActionDropdown, 0, Qt::AlignCenter);
        connect(m_TerrainActionDropdown, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &TerrainShaperWidget::OnTerrainActionDropdownChanged);

        setLayout(mainLayout);
    }

    void TerrainShaperWidget::OnTerrainRefreshButtonClicked()
    {
        // Clear previous entries
        m_TerrainEntries.clear();
        m_TerrainDropdown->clear();

        m_TerrainEntries = Utils::GetAllTerrains();

        for (const AZ::EntityId& entityId : m_TerrainEntries)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);

            if (entity)
            {
                m_TerrainDropdown->addItem(QString::fromStdString(entity->GetName().c_str()), QVariant::fromValue(entityId));
            }
            else
            {
                m_TerrainDropdown->addItem(QString("Terrain %llu").arg(static_cast<AZ::u64>(entityId)), QVariant::fromValue(entityId));
            }
        }
    }

    void TerrainShaperWidget::OnTerrainDropdownChanged(int index)
    {
        if (index < 0 || index >= m_TerrainEntries.size())
            return;

        AZ::EntityId selectedEntityId = m_TerrainEntries[index];

        AZ_Printf("O3DE", "Selected Terrain Entity ID: %llu", static_cast<AZ::u64>(selectedEntityId));
    }

    void TerrainShaperWidget::OnTerrainActionDropdownChanged(int index)
    {
        AZ_Printf("O3DE", "Selected terrain option: %d", index);

        switch (index)
        {
        case 0:
            // TerrainShaperUtils::FlattenTerrain();
            break;
        case 1:
            // TerrainShaperUtils::RaiseTerrain();
            break;
        case 2:
            // TerrainShaperUtils::LowerTerrain();
            break;
        case 3:
            // TerrainShaperUtils::SmoothTerrain();
            break;
        default:
            AZ_Printf("O3DE", "Invalid selection");
            break;
        }
    }
}

#include <moc_TerrainShaperWidget.cpp>
