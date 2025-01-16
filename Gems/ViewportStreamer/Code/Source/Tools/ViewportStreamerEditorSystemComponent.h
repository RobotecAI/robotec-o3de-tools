#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>
#include <Clients/ViewportStreamerSystemComponent.h>

namespace ViewportStreamer
{
    /// System component for ViewportStreamer editor
    class ViewportStreamerEditorSystemComponent
        : public ViewportStreamerSystemComponent
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ViewportStreamerEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ViewportStreamerEditorSystemComponent();
        ~ViewportStreamerEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // EditorEntityContextNotificationBus overrides
        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditor() override;
    };
} // namespace ViewportStreamer
