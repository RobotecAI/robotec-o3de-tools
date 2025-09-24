#pragma once

#include <AzCore/Component/Component.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>

namespace SimpleLidarSensor
{
    class SimpleLidarSensorSystemComponent
        : public AZ::Component
    {
    public:
        AZ_COMPONENT(SimpleLidarSensorSystemComponent, SimpleLidarSensorSystemComponentTypeId);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        SimpleLidarSensorSystemComponent();
        ~SimpleLidarSensorSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

    private:
        void InitPassTemplateMappingsHandler();
        void LoadPassTemplateMappings();
        AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler m_loadTemplatesHandler;
    };

} // namespace SimpleLidarSensor