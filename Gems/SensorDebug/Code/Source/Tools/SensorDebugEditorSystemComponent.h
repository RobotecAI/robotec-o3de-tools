
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <AzToolsFramework/Entity/EditorEntityContextBus.h>
#include <Clients/SensorDebugSystemComponent.h>
namespace SensorDebug
{
    /// System component for SensorDebug editor
    class SensorDebugEditorSystemComponent
        : public SensorDebugSystemComponent
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
        using BaseSystemComponent = SensorDebugSystemComponent;

    public:
        AZ_COMPONENT_DECL(SensorDebugEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        SensorDebugEditorSystemComponent() = default;
        ~SensorDebugEditorSystemComponent() = default;

    private:
        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

        // EditorEntityContextNotificationBus overrides
        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditor() override;
    };
} // namespace SensorDebug
