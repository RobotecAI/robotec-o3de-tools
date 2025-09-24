#include "SimpleLidarSensorEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>

namespace SimpleLidarSensor
{
    AZ_COMPONENT_IMPL(
        SimpleLidarSensorEditorSystemComponent, "SimpleLidarSensorEditorSystemComponent", SimpleLidarSensorEditorSystemComponentTypeId, BaseSystemComponent);

    void SimpleLidarSensorEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimpleLidarSensorEditorSystemComponent, SimpleLidarSensorSystemComponent>()->Version(1)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AZStd::vector<AZ::Crc32>({ AZ_CRC("AssetBuilder") }));
        }
    }

    SimpleLidarSensorEditorSystemComponent::SimpleLidarSensorEditorSystemComponent()
    {
    }

    SimpleLidarSensorEditorSystemComponent::~SimpleLidarSensorEditorSystemComponent()
    {
    }

    void SimpleLidarSensorEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC("SimpleLidarSensorEditorService"));
    }

    void SimpleLidarSensorEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC("SimpleLidarSensorEditorService"));
    }

    void SimpleLidarSensorEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SimpleLidarSensorEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void SimpleLidarSensorEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
        BaseSystemComponent::Activate();
    }

    void SimpleLidarSensorEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void SimpleLidarSensorEditorSystemComponent::OnStartPlayInEditorBegin()
    {
        //BaseSystemComponent::Activate();
    }
    void SimpleLidarSensorEditorSystemComponent::OnStopPlayInEditor()
    {
        //BaseSystemComponent::Deactivate();
    }
} // namespace SimpleLidarSensor