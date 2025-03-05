#include "FPSProfilerEditorSystemComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace FPSProfiler
{
    void FPSProfilerEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        Config::FPSProfilerConfigFile::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerEditorSystemComponent, EditorComponentBase>()
                ->Version(0)
                ->Field("m_Configuration", &FPSProfilerEditorSystemComponent::m_configuration)
                ->Field("m_profileOnGameStart", &FPSProfilerEditorSystemComponent::m_profileOnGameStart);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<FPSProfilerEditorSystemComponent>("FPS Profiler", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_configuration)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerEditorSystemComponent::m_profileOnGameStart,
                        "Profile On Game Start",
                        "Should system start profiling data instantly after game is launched, or await for other system to activate it?");
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
        entity->CreateComponent<FPSProfilerSystemComponent>(m_configuration, m_profileOnGameStart);
    }
} // namespace FPSProfiler
