// File: ExampleComponent.h
#pragma once

#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Component/TickBus.h>
#include <Atom/RPI.Public/View.h>
#include <Atom/RPI.Public/Scene.h>
namespace SimpleLidarSensor
{
    class SimpleLidar
    : public AZ::Component, private AZ::TickBus::Handler
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
        // TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        float m_value = 0.0f;
        AZStd::vector<AZStd::string> m_passHierarchy;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::Scene* m_scene = nullptr;
        AZStd::string m_pipelineName;
        const AZ::Transform AtomToRos{ AZ::Transform::CreateFromQuaternion(
            AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };

    };
}