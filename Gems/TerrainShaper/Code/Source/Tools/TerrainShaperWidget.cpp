
#include <AzCore/Utils/Utils.h>

#include <QLabel>
#include <QVBoxLayout>

#include "TerrainShaperWidget.h"

#include "Utils/TerrainShaperUtils.h"

#include <QPushButton>

namespace TerrainShaper
{
    TerrainShaperWidget::TerrainShaperWidget(QWidget* parent)
        : QWidget(parent)
    {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);

        QLabel* introLabel = new QLabel(QObject::tr("Terrain Shaping Tool"), this);
        mainLayout->addWidget(introLabel, 0, Qt::AlignTop);

        // Create button and add it to layout
        QPushButton* terrainButton = new QPushButton(QObject::tr("RefreshTerrainList"), this);
        mainLayout->addWidget(terrainButton, 0, Qt::AlignCenter);

        connect(terrainButton, &QPushButton::clicked, this, &TerrainShaperWidget::OnTerrainButtonClicked);

        setLayout(mainLayout);
    }

    void TerrainShaperWidget::OnTerrainButtonClicked()
    {
        Utils::GetAllTerrains();
    }
}

#include <moc_TerrainShaperWidget.cpp>
