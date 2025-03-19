#include "FPSProfilerEditorComponent.h"
#include <Clients/FPSProfilerComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzQtComponents/Components/Widgets/FileDialog.h>
#include <UI/UICore/WidgetHelpers.h>

namespace FPSProfiler
{
    void FPSProfilerEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        Configs::FileSaveSettings::Reflect(context);
        Configs::RecordSettings::Reflect(context);
        Configs::PrecisionSettings::Reflect(context);
        Configs::DebugSettings::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerEditorComponent, EditorComponentBase>()
                ->Version(0)
                ->Field("m_configFileEditor", &FPSProfilerEditorComponent::m_configFile)
                ->Field("m_configRecordEditor", &FPSProfilerEditorComponent::m_configRecord)
                ->Field("m_configPrecisionEditor", &FPSProfilerEditorComponent::m_configPrecision)
                ->Field("m_configDebugEditor", &FPSProfilerEditorComponent::m_configDebug);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<FPSProfilerEditorComponent>("FPS Profiler", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->UIElement(AZ::Edit::UIHandlers::Button, "", "Click to open file dialog.")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Select Csv File Path")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &FPSProfilerEditorComponent::SelectCsvPath)

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorComponent::m_configFile)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorComponent::m_configRecord)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorComponent::m_configPrecision)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorComponent::m_configDebug);
            }
        }
    }

    void FPSProfilerEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("FPSProfilerEditorService"));
    }

    void FPSProfilerEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("FPSProfilerEditorService"));
    }

    void FPSProfilerEditorComponent::Activate()
    {
    }

    void FPSProfilerEditorComponent::Deactivate()
    {
    }

    void FPSProfilerEditorComponent::BuildGameEntity(AZ::Entity* entity)
    {
        if (FPSProfilerComponent* gameComponent = entity->CreateComponent<FPSProfilerComponent>())
        {
            gameComponent->m_configFile = m_configFile;
            gameComponent->m_configRecord = m_configRecord;
            gameComponent->m_configPrecision = m_configPrecision;
            gameComponent->m_configDebug = m_configDebug;
        }
    }

    AZ::u32 FPSProfilerEditorComponent::SelectCsvPath()
    {
        QString fileName = QFileDialog::getSaveFileName(AzToolsFramework::GetActiveWindow(), "Pick a csv file path.", "", "*.csv");

        if (fileName.isEmpty())
        {
            QMessageBox::warning(AzToolsFramework::GetActiveWindow(), "Error", "Please specify file path!", QMessageBox::Ok);
            return AZ::Edit::PropertyRefreshLevels::None;
        }

        // Ensure the file has the .csv extension
        if (!fileName.endsWith(".csv", Qt::CaseInsensitive))
        {
            fileName += ".csv"; // Auto-append .csv if missing
        }

        m_configFile.m_OutputFilename = AZStd::string(fileName.toUtf8().constData());
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }
} // namespace FPSProfiler
