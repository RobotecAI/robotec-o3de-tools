
#include "TerrainShaperWidget.h"

#include "Viewport/ViewportMessages.h"

#include <QCheckBox>
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
        QPushButton* terrainButton = new QPushButton(QObject::tr(""), this);
        terrainButton->setIcon(QIcon(":/TerrainShaper/refresh_icon.svg"));
        terrainButton->setIconSize(QSize(24, 24));
        terrainButton->setFixedSize(QSize(32, 32));
        connect(terrainButton, &QPushButton::clicked, this, &TerrainShaperWidget::OnTerrainRefreshButtonClicked);
        formLayout->addRow(new QLabel(QObject::tr("Load Terrain List: "), this), terrainButton);

        // Outline Checkbox
        QCheckBox* outlineCheckbox = new QCheckBox("", this);
        outlineCheckbox->setChecked(m_TerrainSelectSettings.m_enableOutline);
        // connect(outlineCheckbox, &QCheckBox::stateChanged, this);
        formLayout->addRow(new QLabel(QObject::tr("Enable Outline: "), this), outlineCheckbox);

        // Focus Checkbox
        QCheckBox* focusCheckbox = new QCheckBox("", this);
        focusCheckbox->setChecked(m_TerrainSelectSettings.m_enableFocus);
        // connect(focusCheckbox, &QCheckBox::stateChanged, this);
        formLayout->addRow(new QLabel(QObject::tr("Enable Focus: "), this), focusCheckbox);

        // Available Terrains Dropdown
        m_TerrainDropdown = new QComboBox(this);
        connect(m_TerrainDropdown, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &TerrainShaperWidget::OnTerrainDropdownChanged);
        formLayout->addRow(new QLabel(QObject::tr("Select Terrain: "), this), m_TerrainDropdown);

        // Terrain Actions Dropdown
        m_TerrainActionDropdown = new QComboBox(this);
        m_TerrainActionDropdown->addItem("Flatten Terrain", QVariant::fromValue(Config::ShaperActions::Flatten));
        m_TerrainActionDropdown->addItem("Raise Terrain", QVariant::fromValue(Config::ShaperActions::Raise));
        m_TerrainActionDropdown->addItem("Lower Terrain", QVariant::fromValue(Config::ShaperActions::Lower));
        m_TerrainActionDropdown->addItem("Smooth Terrain", QVariant::fromValue(Config::ShaperActions::Smooth));
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
            { Config::BrushTypes::Circle, ":/TerrainShaper/circle_icon.svg" },
            { Config::BrushTypes::Rectangle, ":/TerrainShaper/rectangle_icon.svg" },
            { Config::BrushTypes::Square, ":/TerrainShaper/square_icon.svg" },
            { Config::BrushTypes::Triangle, ":/TerrainShaper/triangle_icon.svg" }
        };

        for (const Config::BrushInfo& brush : brushes)
        {
            QPushButton* button = new QPushButton(this);
            button->setCheckable(true);
            button->setProperty("brushType", QVariant::fromValue(brush.m_brushType)); // Store enum in button

            QIcon icon((brush.m_iconPath.c_str()));
            button->setIcon(icon);
            button->setIconSize(QSize(24, 24));
            button->setFixedSize(32, 32);

            m_BrushButtonGroup->addButton(button, static_cast<int>(brush.m_brushType));
            m_BrushLayout->addWidget(button, 0, Qt::AlignCenter | Qt::AlignLeft);

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

        AZ_Printf("TerrainShaperWidget::OnTerrainDropdownChanged()", "Selected Terrain Entity ID: %llu\t%b\t%b",
            m_TerrainEntries[index], m_TerrainSelectSettings.m_enableOutline, m_TerrainSelectSettings.m_enableFocus);

        // Select the entity
        if (m_TerrainSelectSettings.m_enableOutline)
        {
            AzToolsFramework::EntityIdList entities = { m_TerrainEntries[index] };
            AzToolsFramework::ToolsApplicationRequestBus::Broadcast(
                &AzToolsFramework::ToolsApplicationRequests::SetSelectedEntities, entities);
        }

        // Focus the camera on the selected entity
        if (m_TerrainSelectSettings.m_enableFocus)
        {
            AzToolsFramework::EditorRequestBus::Broadcast(
                &AzToolsFramework::EditorRequestBus::Events::GoToSelectedEntitiesInViewports);
        }
    }

    void TerrainShaperWidget::OnTerrainActionDropdownChanged(int index)
    {
        if (index < 0)
            return;

        // Retrieve selected action from QVariant
        QVariant actionVariant = m_TerrainActionDropdown->itemData(index);
        Config::ShaperActions selectedAction = actionVariant.value<Config::ShaperActions>();

        switch (selectedAction)
        {
        case Config::ShaperActions::Flatten:
            // TerrainShaperUtils::FlattenTerrain();
            break;
        case Config::ShaperActions::Raise:
            // TerrainShaperUtils::RaiseTerrain();
            break;
        case Config::ShaperActions::Lower:
            // TerrainShaperUtils::LowerTerrain();
            break;
        case Config::ShaperActions::Smooth:
            // TerrainShaperUtils::SmoothTerrain();
            break;
        default:
            AZ_Printf("TerrainShaperWidget::OnTerrainActionDropdownChanged()", "Invalid selection.");
            break;
        }

        AZ_Printf("TerrainShaperWidget::OnTerrainActionDropdownChanged()", "Selected Action: %d", index);
    }

    void TerrainShaperWidget::OnBrushSelected(int index)
    {
        // Convert id back to enum
        m_SelectedBrush = static_cast<Config::BrushTypes>(index);

        // Print selected brush type
        QString brushName;
        switch (m_SelectedBrush)
        {
            case Config::BrushTypes::Circle:
                brushName = "Circle";
                break;
            case Config::BrushTypes::Rectangle:
                brushName = "Rectangle";
                break;
            case Config::BrushTypes::Square:
                brushName = "Square";
                break;
            case Config::BrushTypes::Triangle:
                brushName = "Triangle";
                break;
            default:
                brushName = "Unknown";
                break;
        }

        AZ_Printf("TerrainShaperWidget::OnBrushSelected()", "Selected Brush Type: %s", brushName.toUtf8().constData());
    }
}

#include <moc_TerrainShaperWidget.cpp>
