// File: ExampleComponent.h
#pragma once

#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/SerializeContext.h>


namespace SimpleLidarSensor
{
    class SimpleLidar
    : public AZ::Component
    {
    public:
        AZ_COMPONENT(SimpleLidar, SimpleLidarComponentTypeId);
        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);


    private:
        float m_value = 0.0f;
    };
}