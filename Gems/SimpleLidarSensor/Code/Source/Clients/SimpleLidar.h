// File: ExampleComponent.h
#pragma once

#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Component/TickBus.h>
#include <Atom/RPI.Public/View.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <opencv2/opencv.hpp>
namespace SimpleLidarSensor
{
    static constexpr unsigned int ViewCount = 6;
    struct PendingFrames
    {
        AZStd::unordered_map<unsigned int, cv::Mat> m_viewsDataColor;
        AZStd::unordered_map<unsigned int, cv::Mat> m_viewsDataDepth;
        bool IsComplete()
        {
            return m_viewsDataDepth.size() == ViewCount && m_viewsDataColor.size() == ViewCount;
        }

        void ReportDepthFrameCaptured(unsigned int frameId, const cv::Mat& frame)
        {
            frame.copyTo(m_viewsDataDepth[frameId]);
        }
        void ReportColorFrameCaptured(unsigned int frameId, const cv::Mat& frame)
        {
            frame.copyTo(m_viewsDataColor[frameId]);
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
        void FrameComplete(const PendingFrames& completedFrame);

        // TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        float m_value = 0.0f;

        static constexpr float HorizontalFOV = 360.0f / ViewCount;
        AZ::Matrix3x3 m_cameraMatrix; //! Classical camera intrinsics matrix, the same for all cameras
        AZStd::vector<AZ::Transform> m_cameraToLidarCoordinate; //! directions for each camera in rig space (Z forward, X right, Y down)
        AZStd::vector<AZStd::vector<AZStd::string>> m_passHierarchies;
        AZStd::vector<AZ::RPI::RenderPipelinePtr> m_pipelines;
        AZStd::vector<AZStd::string> m_pipelineNames;
        AZStd::vector<AZ::RPI::ViewPtr> m_view;
        AZ::RPI::Scene* m_scene = nullptr;

        AZStd::mutex m_mutex;
        AZStd::map< AZStd::chrono::steady_clock::time_point, PendingFrames> m_pendingFrames; // cache of frames indexed by time of request

    };
}