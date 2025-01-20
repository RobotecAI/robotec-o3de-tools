#include "ViewportStreamerEditorSystemComponent.h"
#include "Clients/ViewportStreamerSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ViewportStreamer/ViewportStreamerTypeIds.h>

namespace ViewportStreamer
{
    AZ_COMPONENT_IMPL(
        ViewportStreamerEditorSystemComponent,
        "ViewportStreamerEditorSystemComponent",
        ViewportStreamerEditorSystemComponentTypeId,
        ViewportStreamerSystemComponent);

    void ViewportStreamerEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ViewportStreamerEditorSystemComponent, ViewportStreamerSystemComponent>()->Version(0);
        }
    }

    ViewportStreamerEditorSystemComponent::ViewportStreamerEditorSystemComponent() = default;

    ViewportStreamerEditorSystemComponent::~ViewportStreamerEditorSystemComponent() = default;

    void ViewportStreamerEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        ViewportStreamerSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ViewportStreamerEditorService"));
    }

    void ViewportStreamerEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        ViewportStreamerSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ViewportStreamerEditorService"));
    }

    void ViewportStreamerEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        ViewportStreamerSystemComponent::GetRequiredServices(required);
    }

    void ViewportStreamerEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void ViewportStreamerEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void ViewportStreamerEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        AZ::TickBus::QueueFunction(
            [this]()
            {
                ViewportStreamerSystemComponent::Activate();
            });
    }

    void ViewportStreamerEditorSystemComponent::OnStopPlayInEditor()
    {
        ViewportStreamerSystemComponent::Deactivate();
    }

} // namespace ViewportStreamer
