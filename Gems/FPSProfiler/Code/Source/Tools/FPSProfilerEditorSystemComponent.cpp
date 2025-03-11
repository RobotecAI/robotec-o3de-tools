#include "FPSProfilerEditorSystemComponent.h"

#include <Clients/FPSProfilerSystemComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace FPSProfiler
{
    void FPSProfilerEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        Configs::FileSaveSettings::Reflect(context);
        Configs::RecordSettings::Reflect(context);
        Configs::PrecisionSettings::Reflect(context);
        Configs::DebugSettings::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerEditorSystemComponent, EditorComponentBase>()
                ->Version(0)
                ->Field("m_configFile", &FPSProfilerEditorSystemComponent::m_configFile)
                ->Field("m_configRecord", &FPSProfilerEditorSystemComponent::m_configRecord)
                ->Field("m_configPrecision", &FPSProfilerEditorSystemComponent::m_configPrecision)
                ->Field("m_configDebug", &FPSProfilerEditorSystemComponent::m_configDebug);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<FPSProfilerEditorSystemComponent>("FPS Profiler", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_configFile)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_configRecord)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_configPrecision)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_configDebug);
            }
        }
    }

    void FPSProfilerEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("FPSProfilerEditorService"));
    }

    void FPSProfilerEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("FPSProfilerEditorService"));
    }

    void FPSProfilerEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void FPSProfilerEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
    }

    void FPSProfilerEditorSystemComponent::BuildGameEntity(AZ::Entity* entity)
    {
        entity->CreateComponent<FPSProfilerSystemComponent>(m_configFile, m_configRecord, m_configPrecision, m_configDebug);
    }
} // namespace FPSProfiler
