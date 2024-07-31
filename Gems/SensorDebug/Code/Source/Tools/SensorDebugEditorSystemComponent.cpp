
#include "SensorDebugEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SensorDebug/SensorDebugTypeIds.h>

namespace SensorDebug
{
    AZ_COMPONENT_IMPL(
        SensorDebugEditorSystemComponent, "SensorDebugEditorSystemComponent", SensorDebugEditorSystemComponentTypeId, BaseSystemComponent);

    void SensorDebugEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SensorDebugEditorSystemComponent, SensorDebugSystemComponent>()->Version(0);
        }
    }

    void SensorDebugEditorSystemComponent::Activate()
    {
        SensorDebugSystemComponent::Activate();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void SensorDebugEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
        SensorDebugSystemComponent::Deactivate();
    }

    void SensorDebugEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        ClearSensors();
    }

    void SensorDebugEditorSystemComponent::OnStopPlayInEditor()
    {
        ClearSensors();
    }
} // namespace SensorDebug
