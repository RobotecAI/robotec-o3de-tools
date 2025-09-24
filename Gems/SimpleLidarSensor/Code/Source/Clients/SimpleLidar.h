
#pragma once

#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/View.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include "LidarConfiguration.h"
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <SimpleLidarSensor/SimpleLidarSensorTypeIds.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace SimpleLidarSensor
{
    static constexpr unsigned int ViewCount = 4; // Number of cameras in the rig
    struct PendingFrames
    {
        AZStd::unordered_map<unsigned int, cv::Mat> m_viewsDataColor;
        AZStd::unordered_map<unsigned int, cv::Mat> m_viewsDataDepth;

        // Rule of 3: Destructor, Copy Constructor, Assignment Operator
        ~PendingFrames() = default;

        // Copy constructor
        PendingFrames(const PendingFrames& other)
        {
            for (const auto& [id, mat] : other.m_viewsDataColor)
            {
                mat.copyTo(m_viewsDataColor[id]);
            }
            for (const auto& [id, mat] : other.m_viewsDataDepth)
            {
                mat.copyTo(m_viewsDataDepth[id]);
            }
        }

        // Assignment operator
        PendingFrames& operator=(const PendingFrames& other)
        {
            if (this != &other)
            {
                m_viewsDataColor.clear();
                m_viewsDataDepth.clear();

                for (const auto& [id, mat] : other.m_viewsDataColor)
                {
                    mat.copyTo(m_viewsDataColor[id]);
                }
                for (const auto& [id, mat] : other.m_viewsDataDepth)
                {
                    mat.copyTo(m_viewsDataDepth[id]);
                }
            }
            return *this;
        }

        // Move constructor and move assignment (Rule of 5)
        PendingFrames(PendingFrames&& other) noexcept
            : m_viewsDataColor(AZStd::move(other.m_viewsDataColor))
            , m_viewsDataDepth(AZStd::move(other.m_viewsDataDepth))
        {
        }

        PendingFrames& operator=(PendingFrames&& other) noexcept
        {
            if (this != &other)
            {
                m_viewsDataColor = AZStd::move(other.m_viewsDataColor);
                m_viewsDataDepth = AZStd::move(other.m_viewsDataDepth);
            }
            return *this;
        }

        // Default constructor
        PendingFrames() = default;

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
        : public ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>
    {
    public:
        AZ_COMPONENT(SimpleLidar, SimpleLidarComponentTypeId);
        static void Reflect(AZ::ReflectContext* context);

        SimpleLidar();

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);


    private:
        void PublishPointCloud(const PendingFrames& completedFrame);


        void ImageCallback(const AZStd::chrono::steady_clock::time_point& requestTimestamp ,unsigned int viewIndex, const AZ::RPI::AttachmentReadback::ReadbackResult& result);
        void OnSensorTick();
        float m_value = 0.0f;

        AZ::Matrix3x3 m_cameraMatrix; //! Classical camera intrinsics matrix, the same for all cameras
        AZStd::vector<AZ::Transform> m_cameraToLidarCoordinate; //! directions for each camera in rig space (Z forward, X right, Y down)
        AZStd::vector<AZStd::vector<AZStd::string>> m_passHierarchies;
        AZStd::vector<AZ::RPI::RenderPipelinePtr> m_pipelines;
        AZStd::vector<AZStd::string> m_pipelineNames;
        AZStd::vector<AZ::RPI::ViewPtr> m_view;
        AZ::RPI::Scene* m_scene = nullptr;

        AZStd::mutex m_mutex;
        AZStd::map< AZStd::chrono::steady_clock::time_point, PendingFrames> m_pendingFrames; // cache of frames indexed by time of request

        // ROS2 publisher for point cloud
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_debugImagePublisher;

        AZStd::optional<size_t> m_rayCount;

        // Lidar configuration
        LidarConfiguration m_lidarConfiguration;
    };
}