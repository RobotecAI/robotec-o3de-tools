// File: ExampleComponent.h
#pragma once

#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Component/TickBus.h>
#include <Atom/RPI.Public/View.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>
namespace SimpleLidarSensor
{
    struct PendingFrames
    {
        double m_timeStamp = 0.0;
        AZStd::unordered_set<unsigned int> m_framesIdToCapture;
        AZStd::unordered_set<unsigned int> m_capturedIdFrames;
        bool IsComplete()
        {
            return m_framesIdToCapture.size() == m_capturedIdFrames.size();
        }

        void ReportFrameCaptured(unsigned int frameId, double timeStamp = 0.0)
        {
            if (m_framesIdToCapture.contains(frameId) && timeStamp == m_timeStamp)
            {
                m_capturedIdFrames.insert(frameId);
            }
            else
            {
                AZ_Warning("SimpleLidar", false, "Received unexpected frameId %u or timestamp %f (expected %f)", frameId, timeStamp, m_timeStamp);
            }
        }

        void Reset()
        {
            m_framesIdToCapture.clear();
            m_capturedIdFrames.clear();
        }
    };
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
        static constexpr unsigned int ViewCount = 6;
        static constexpr float HorizontalFOV = 360.0f / ViewCount;
        AZStd::vector<AZStd::vector<AZStd::string>> m_passHierarchies;
        AZStd::vector<AZ::RPI::RenderPipelinePtr> m_pipelines;
        AZStd::vector<AZ::RPI::ViewPtr> m_view;
        AZ:: RPI::AttachmentReadback::CallbackFunction m_callback;
        AZ::RPI::Scene* m_scene = nullptr;





        const AZ::Transform AtomToRos{ AZ::Transform::CreateFromQuaternion(
            AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };
        PendingFrames m_pendingFrames;
    };
}