#pragma once

#include <AzCore/Component/Component.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include "Clients/SimpleLidarSensorSystemComponent.h"
#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

namespace SimpleLidarSensor
{
    /// System component for SimpleLidarSensor editor
    class SimpleLidarSensorEditorSystemComponent
        : public SimpleLidarSensorSystemComponent
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
        using BaseSystemComponent = SimpleLidarSensorSystemComponent;

    public:
        AZ_COMPONENT_DECL(SimpleLidarSensorEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        SimpleLidarSensorEditorSystemComponent();
        ~SimpleLidarSensorEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // EditorEntityContextNotificationBus overrides
        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditor() override;
    };

} // namespace SimpleLidarSensor