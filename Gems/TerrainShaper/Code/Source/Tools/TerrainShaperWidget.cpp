
#include "TerrainShaperWidget.h"

#include <Utils/TerrainShaperUtils.h>
#include <AzCore/Utils/Utils.h>

#include <QFormLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>

namespace TerrainShaper
{
    TerrainShaperWidget::TerrainShaperWidget(QWidget* parent)
        : QWidget(parent)
    {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);

        // Title
        QLabel* introLabel = new QLabel(QObject::tr("Terrain Shaping Tool"), this);
        introLabel->setStyleSheet("font-weight: bold; font-size: 14px;");
        mainLayout->addWidget(introLabel, 0, Qt::AlignTop | Qt::AlignLeft);

        // Form Layout for Labels and Widgets
        QFormLayout* formLayout = new QFormLayout();

        // Refresh Available Terrains Button
        QPushButton* terrainButton = new QPushButton(QObject::tr("Refresh"), this);
        terrainButton->setIcon(QIcon(":/TerrainShaper/refresh_icon.svg"));
        connect(terrainButton, &QPushButton::clicked, this, &TerrainShaperWidget::OnTerrainRefreshButtonClicked);
        formLayout->addRow(new QLabel(QObject::tr("Available Terrains: "), this), terrainButton);

        // Available Terrains Dropdown
        m_TerrainDropdown = new QComboBox(this);
        connect(m_TerrainDropdown, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &TerrainShaperWidget::OnTerrainDropdownChanged);
        formLayout->addRow(new QLabel(QObject::tr("Select Terrain: "), this), m_TerrainDropdown);

        // Terrain Actions Dropdown
        m_TerrainActionDropdown = new QComboBox(this);
        m_TerrainActionDropdown->addItem("Flatten Terrain", QVariant::fromValue(Config::TerrainShaperActions::Flatten));
        m_TerrainActionDropdown->addItem("Raise Terrain", QVariant::fromValue(Config::TerrainShaperActions::Raise));
        m_TerrainActionDropdown->addItem("Lower Terrain", QVariant::fromValue(Config::TerrainShaperActions::Lower));
        m_TerrainActionDropdown->addItem("Smooth Terrain", QVariant::fromValue(Config::TerrainShaperActions::Smooth));
        connect(m_TerrainActionDropdown, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &TerrainShaperWidget::OnTerrainActionDropdownChanged);
        formLayout->addRow(new QLabel(QObject::tr("Select Action: "), this), m_TerrainActionDropdown);

        // Create a row of brush buttons
        CreateBrushSelect();
        formLayout->addRow(new QLabel(QObject::tr("Pick Brush: "), this), m_BrushLayout);

        mainLayout->addLayout(formLayout);
        setLayout(mainLayout);
    }

    void TerrainShaperWidget::CreateBrushSelect()
    {
        m_BrushLayout = new QHBoxLayout();
        m_BrushButtonGroup = new QButtonGroup(this);

        Config::BrushInfo brushes[] = {
            { Config::TerrainShaperBrushTypes::Circle, ":/TerrainShaper/circle_icon.svg" },
            { Config::TerrainShaperBrushTypes::Rectangle, ":/TerrainShaper/rectangle_icon.svg" },
            { Config::TerrainShaperBrushTypes::Square, ":/TerrainShaper/square_icon.svg" },
            { Config::TerrainShaperBrushTypes::Triangle, ":/TerrainShaper/triangle_icon.svg" }
        };

        for (const Config::BrushInfo& brush : brushes)
        {
            QPushButton* button = new QPushButton(this);
            button->setCheckable(true);
            button->setProperty("brushType", QVariant::fromValue(brush.m_brushType)); // Store enum in button

            // Load SVG from Resource File
            QIcon icon((brush.m_iconPath.c_str()));
            button->setIcon(icon);
            // button->setIconSize(QSize(24, 24));  // Set icon size

            // Remove text and keep only icon
            // button->setFixedSize(32, 32);  // Ensure buttons have proper size

            m_BrushButtonGroup->addButton(button, static_cast<int>(brush.m_brushType));
            m_BrushLayout->addWidget(button);

            // Default selected brush
            if (brush.m_brushType == m_SelectedBrush)
            {
                button->setChecked(true);
            }
        }

        connect(m_BrushButtonGroup, &QButtonGroup::idClicked,
                this, &TerrainShaperWidget::OnBrushSelected);

        // Style for highlighted selection
        setStyleSheet(R"(
        QPushButton {
            padding: 4px;
            border: 2px solid transparent;
            border-radius: 5px;
            background-color: #444;
        }
        QPushButton:checked {
            border: 2px solid blue; /* Highlight selected brush */
            background-color: #555;
        })");
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
        if (index < 0)
            return;

        // Retrieve selected action from QVariant
        QVariant actionVariant = m_TerrainActionDropdown->itemData(index);
        Config::TerrainShaperActions selectedAction = actionVariant.value<Config::TerrainShaperActions>();

        switch (selectedAction)
        {
        case Config::TerrainShaperActions::Flatten:
            // TerrainShaperUtils::FlattenTerrain();
            break;
        case Config::TerrainShaperActions::Raise:
            // TerrainShaperUtils::RaiseTerrain();
            break;
        case Config::TerrainShaperActions::Lower:
            // TerrainShaperUtils::LowerTerrain();
            break;
        case Config::TerrainShaperActions::Smooth:
            // TerrainShaperUtils::SmoothTerrain();
            break;
        default:
            AZ_Printf("OnTerrainActionDropdownChanged", "Invalid selection");
            break;
        }

        AZ_Printf("OnTerrainActionDropdownChanged", "Selected Action: %d", index);
    }

    void TerrainShaperWidget::OnBrushSelected(int index)
    {
        // Convert id back to enum
        m_SelectedBrush = static_cast<Config::TerrainShaperBrushTypes>(index);

        // Print selected brush type
        QString brushName;
        switch (m_SelectedBrush)
        {
        case Config::TerrainShaperBrushTypes::Circle: brushName = "Circle"; break;
        case Config::TerrainShaperBrushTypes::Rectangle: brushName = "Rectangle"; break;
        case Config::TerrainShaperBrushTypes::Square: brushName = "Square"; break;
        case Config::TerrainShaperBrushTypes::Triangle: brushName = "Triangle"; break;
        default: brushName = "Unknown"; break;
        }

        AZ_Printf("O3DE", "Selected Brush Type: %s", brushName.toUtf8().constData());
    }
}

#include <moc_TerrainShaperWidget.cpp>
