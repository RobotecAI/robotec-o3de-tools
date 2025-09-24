#include "SimpleLidar.h"
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include "AzCore/Math/Matrix4x4.h"
#include <Atom/Feature/PostProcess/PostProcessFeatureProcessorInterface.h>

#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MatrixUtils.h>
#include <fstream>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "BilinearSampling.h"
namespace SimpleLidarSensor
{

    namespace
    {

        // type aliases for point cloud iterators - can be changed to modify point cloud data types,
        using PointcloudXYZType = float;
        using PointcloudIntensityType = float;
        using PointcloudRingType = uint16_t;
        using PointcloudTimeType = float;
        using PointcloudRGBType = float; // RGB is stored as float, but iterated as uint8_t

        constexpr uint8_t PointcloudXYZTypeId = 7;         // FLOAT32
        constexpr uint8_t PointcloudIntensityTypeId = 7;   // FLOAT32
        constexpr uint8_t PointcloudRingTypeId = 4;        // UINT16
        constexpr uint8_t PointcloudRGBTypeId = 7;         // FLOAT32 -

        using PointcloudXYZIterator = sensor_msgs::PointCloud2Iterator<PointcloudXYZType>;
        using PointcloudIntensityIterator = sensor_msgs::PointCloud2Iterator<PointcloudIntensityType>;
        using PointcloudRingIterator = sensor_msgs::PointCloud2Iterator<PointcloudRingType>;
        using PointcloudTimeIterator = sensor_msgs::PointCloud2Iterator<PointcloudTimeType>;
        using PointcloudColorIterator = sensor_msgs::PointCloud2Iterator<uint8_t>;

        using OptionalIntensityIterator = AZStd::optional<PointcloudIntensityIterator>;
        using OptionalRingIterator = AZStd::optional<PointcloudRingIterator>;
        using OptionalTimeIterator = AZStd::optional<PointcloudTimeIterator>;
        using OptionalColorIterator = AZStd::optional<PointcloudColorIterator>;


        float GetAspectRatio(float width, float height)
        {
            return width / height;
        };

        AZ::Matrix3x3 MakeCameraIntrinsics(int width, int height, float horizontalFoV)
        {
            const auto w = static_cast<float>(width);
            const auto h = static_cast<float>(height);
            const float verticalFieldOfView = 2.0 * AZStd::atan(AZStd::tan(horizontalFoV / 2.0) * GetAspectRatio(height, width));
            const float focalLengthX = w / (2.0 * AZStd::tan(horizontalFoV / 2.0));
            const float focalLengthY = h / (2.0 * AZStd::tan(verticalFieldOfView / 2.0));
            return AZ::Matrix3x3::CreateFromRows({ focalLengthX, 0.f, w / 2.f }, { 0.f, focalLengthY, h / 2.f }, { 0.f, 0.f, 1.f });
        }

        //! Creates clip matrix for Atom
        AZ::Matrix4x4 MakeClipMatrix(int width, int height, float verticalFieldOfView, float nearDist, float farDist)
        {
            AZ::Matrix4x4 localViewToClipMatrix;
            AZ::MakePerspectiveFovMatrixRH(
                localViewToClipMatrix, verticalFieldOfView, GetAspectRatio(width, height), nearDist, farDist, true);
            return localViewToClipMatrix;
        }

        //! Returns a transformation matrix (rotation only) for given view index
        AZ::Transform GetViewTransform(int viewIndex, float horizontalFOV)
        {
            const float angle = viewIndex * horizontalFOV;
            const AZ::Quaternion localRot = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), -angle);
            return AZ::Transform::CreateFromQuaternion(localRot);
        }

        //! Rotate whole camera optical coordinate system (Z forward, Y down) to World (X forward, Z up)
        AZ::Transform GetCameraRigRotation()
        {
            const auto matrix =
                AZ::Matrix3x3::CreateFromRows(AZ::Vector3::CreateAxisZ(), -AZ::Vector3::CreateAxisX(), -AZ::Vector3::CreateAxisY());
            return AZ::Transform::CreateFromMatrix3x3(matrix);
        }

        //! Convert planar depth (depth along the camera Z axis) to range (radial distance from the sensor origin)
        float PlanarDepthToRange(const AZ::Matrix3x3& cameraMatrix, float u, float v, float planarDepth)
        {
            const float& fx = cameraMatrix.GetElement(0, 0);
            const float& fy = cameraMatrix.GetElement(1, 1);
            const float& cx = cameraMatrix.GetElement(0, 2);
            const float& cy = cameraMatrix.GetElement(1, 2);
            const float uc = u - cx;
            const float vc = v - cy;
            const float range = planarDepth * AZStd::sqrt((uc * uc) / (fx * fx) + (vc * vc) / (fy * fy) + 1.0f);
            return range;
        }

        const char* PointCloudTypeName = "sensor_msgs::msg::PointCloud2";
        const char* ImageTypeName = "sensor_msgs::msg::Image";
    } // namespace

    SimpleLidar::SimpleLidar()
    {
        m_cameraMatrix = AZ::Matrix3x3::CreateIdentity();
        // Set up default ROS2 publisher configuration
        m_sensorConfiguration.m_frequency = 100.f;
        m_sensorConfiguration.m_publishingEnabled = true;

        ROS2::TopicConfiguration pc;
        pc.m_type = PointCloudTypeName;
        pc.m_topic = "point_cloud";
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(pc.m_type, pc));

        ROS2::TopicConfiguration img;
        img.m_type = ImageTypeName;
        img.m_topic = "lidar_debug_image";
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(img.m_type, img));
    }

    // Reflect for serialization and scripting
    void SimpleLidar::Reflect(AZ::ReflectContext* context)
    {
        LidarConfiguration::Reflect(context);

        if (auto serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SimpleLidar, SensorBaseType>()
                ->Version(2)
                ->Field("value", &SimpleLidar::m_value)
                ->Field("LidarConfiguration", &SimpleLidar::m_lidarConfiguration);

            if (auto ec = serialize->GetEditContext())
            {
                ec->Class<SimpleLidar>("SimpleLidar", "SimpleLidar using graphics pipeline")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SimpleLidar::m_lidarConfiguration,
                                 "Lidar Configuration", "Configuration parameters for the LIDAR sensor");
            }
        }
    }

    void SimpleLidar::Activate()
    {
        SensorBaseType::Activate();

        m_pipelines.resize(ViewCount);
        m_view.resize(ViewCount);
        m_passHierarchies.resize(ViewCount);
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        const int width = 1024;
        const int height = 512;
        //const float VerticalFOV = HorizontalFOV / GetAspectRatio(width, height);
        const float HorizontalFOV = 2.0*M_PI / ViewCount; // add some margin to avoid seams
        const float VerticalFOV = 2.0 * AZStd::atan(AZStd::tan(HorizontalFOV / 2.0) * GetAspectRatio(height, width));
        const float VerticalFOVDegHalf = AZ::RadToDeg(VerticalFOV/2.0f);

        // check if we need to limit the number of rays
        AZ_Warning("SimpleLidar", AZStd::abs(m_lidarConfiguration.m_minElevationDeg) < VerticalFOVDegHalf, "Some of the configured vertical FoV is outside of camera FoV");
        AZ_Warning("SimpleLidar", AZStd::abs(m_lidarConfiguration.m_maxElevationDeg) < VerticalFOVDegHalf, "Some of the configured vertical FoV is outside of camera FoV");




        const auto nearDist = m_lidarConfiguration.m_minRange * 0.9f; // add some margin to min range
        const auto farDist = m_lidarConfiguration.m_maxRange * 1.1f; // add some margin to max range
        const AZ::Matrix4x4 localViewToClipMatrix = MakeClipMatrix(width, width, HorizontalFOV, nearDist, farDist);
        m_cameraMatrix = MakeCameraIntrinsics(width, height, HorizontalFOV);

        for (int i = 0; i < ViewCount; ++i)
        {
            auto& pipeline = m_pipelines[i];
            auto& view = m_view[i];
            auto& passHierarchy = m_passHierarchies[i];
            const AZ::Name viewName = AZ::Name("MainCamera");
            view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
            view->SetViewToClipMatrix(localViewToClipMatrix);

            const auto pipelineName = AZStd::string::format("SimpleLidarCamera%s_%d", GetEntityId().ToString().c_str(), i);
            m_pipelineNames.push_back(pipelineName);
            AZ::RPI::RenderPipelineDescriptor pipelineDesc;
            pipelineDesc.m_mainViewTagName = "MainCamera";

            pipelineDesc.m_allowModification = false;
            pipelineDesc.m_name = pipelineName;
            pipelineDesc.m_renderSettings.m_multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();
            pipelineDesc.m_rootPassTemplate = "PipelineRenderToTextureLidarColor";
            pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
            pipeline->RemoveFromRenderTick();
            if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(pipeline->GetRootPass().get()))
            {
                renderToTexturePass->ResizeOutput(width, height);
            }
            m_scene->AddRenderPipeline(pipeline);

            passHierarchy.push_back(pipelineName);
            passHierarchy.push_back("CopyToSwapChain");
            //
            pipeline->SetDefaultView(view);
            if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessorInterface>())
            {
                const AZ::RPI::ViewPtr targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
                fp->SetViewAlias(view, targetView);
            }

            const auto viewTransform = GetViewTransform(i, HorizontalFOV) * GetCameraRigRotation();
            m_cameraToLidarCoordinate.push_back(viewTransform);
        }

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "ROS2 node is not initialized");

        {
            // Set up ROS2 publisher for point cloud
            const auto& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[PointCloudTypeName];
            AZStd::string fullTopic;
            ROS2::ROS2NamesRequestBus::BroadcastResult(
                fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), publisherConfig.m_topic);


            m_pointCloudPublisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic.data(), publisherConfig.GetQoS());
        }

        if (m_lidarConfiguration.m_publishDebugImages)
        {
            // Set up ROS2 publisher for point cloud
            const auto& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[ImageTypeName];
            AZStd::string fullTopic;
            ROS2::ROS2NamesRequestBus::BroadcastResult(
                fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), publisherConfig.m_topic);
            m_debugImagePublisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(fullTopic.data(), publisherConfig.GetQoS());
        }

        // Start the sensor with configured frequency
        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                OnSensorTick();
            });
    }

    void SimpleLidar::Deactivate()
    {
        StopSensor();
        m_pointCloudPublisher.reset();
        SensorBaseType::Deactivate();
    }

    void SimpleLidar::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("SimpleLidar"));
    }

    void SimpleLidar::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("SimpleLidar"));
    }

    void SimpleLidar::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }


    void SimpleLidar::ImageCallback(const AZStd::chrono::steady_clock::time_point& requestTimestamp ,unsigned int viewIndex, const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        const AZStd::unordered_map<AZ::RHI::Format, int> FormatToCvFormat = { { AZ::RHI::Format::R8G8B8A8_UNORM, CV_8UC4 },
                                                                                    { AZ::RHI::Format::R32_FLOAT, CV_32F } };
        if (result.m_state != AZ::RPI::AttachmentReadback::ReadbackState::Success)
        {
            return;
        }

        PendingFrames completedFrames;
        {
            AZStd::unique_lock<AZStd::mutex> lock(m_mutex);

            const auto it = m_pendingFrames.find(requestTimestamp);
            if (it == m_pendingFrames.end())
            {
                return;
            }

            auto& pendingFrame = it->second;

            // convert to cv::Mat
            const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
            const auto format = descriptor.m_format;

            auto formatIt = FormatToCvFormat.find(format);
            AZ_Assert(formatIt != FormatToCvFormat.end(), "Unexpected format in result %u", static_cast<uint32_t>(format));
            if (formatIt != FormatToCvFormat.end())
            {
                const int width = descriptor.m_size.m_width;
                const int height = descriptor.m_size.m_height;
                auto cvFormat = formatIt->second;
                cv::Mat frame(height, width, cvFormat, (void*)result.m_dataBuffer->data());
                const bool isDepth = frame.channels() == 1;
                if (isDepth)
                {
                    pendingFrame.ReportDepthFrameCaptured(viewIndex, frame);
                }
                else
                {
                    pendingFrame.ReportColorFrameCaptured(viewIndex, frame);
                }


                if (pendingFrame.IsComplete())
                {
                    AZStd::swap(it->second, completedFrames);
                    m_pendingFrames.erase(requestTimestamp);
                }
            }
        }
        if (completedFrames.IsComplete())
        {
            PublishPointCloud(completedFrames);
        }

    }

    void SimpleLidar::OnSensorTick()
    {
        const auto time = AZStd::chrono::steady_clock::now();

        //! Coordinate system conversion from O3DE/Atom to OpenCV (Z forward, X right, -Y down)
        const AZ::Transform AtomToCv{ AZ::Transform::CreateFromQuaternion(
            AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };

        const auto EntityPose = GetEntity()->GetTransform()->GetWorldTM();
        const auto EntityPoseNoScaling =
            AZ::Transform::CreateFromQuaternionAndTranslation(EntityPose.GetRotation(), EntityPose.GetTranslation());

        // house-keeping for pending frames
        while (m_pendingFrames.size() > 3)
        {
            if (!m_pendingFrames.begin()->second.IsComplete())
            {
                const double ts = AZStd::chrono::duration<double>(m_pendingFrames.begin()->first.time_since_epoch()).count();
                AZ_Warning("SimpleLidar", false, "Dropping non-completed pending frame %d", ts);
            }

            m_pendingFrames.erase(m_pendingFrames.begin());
        }
        for (int i = 0; i < ViewCount; ++i)
        {
            auto& pipeline = m_pipelines[i];
            auto& view = m_view[i];
            const auto& passHierarchy = m_passHierarchies[i];
            const auto& viewTransform = m_cameraToLidarCoordinate[i];
            const auto& pipelineName = m_pipelineNames[i];

            const AZ::Transform cameraPose = (EntityPoseNoScaling * viewTransform * AtomToCv).GetInverse();
            view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromTransform(cameraPose));
            pipeline->AddToRenderTickOnce();
            AZ::Render::FrameCaptureOutcome captureOutcome;

            auto callback = [this, viewIndex = i, ts = time](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                if (result.m_state == AZ::RPI::AttachmentReadback::ReadbackState::Success)
                {
                    ImageCallback(ts, viewIndex, result);
                }
                else
                {
                    AZ_Error("SimpleLidar", false, "Capture %d Failed", viewIndex);
                }
            };

            AZ::Render::FrameCaptureRequestBus::BroadcastResult(
                captureOutcome,
                &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
                callback,
                passHierarchy,
                AZStd::string("Output"),
                AZ::RPI::PassAttachmentReadbackOption::Output);

            AZ::Render::FrameCaptureOutcome captureOutcomeDepth;
            AZStd::vector<AZStd::string> passHierarchyDepth{ pipelineName, "DepthPrePass" };
            AZ::Render::FrameCaptureRequestBus::BroadcastResult(
                captureOutcomeDepth,
                &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
                callback,
                passHierarchyDepth,
                AZStd::string("DepthLinear"),
                AZ::RPI::PassAttachmentReadbackOption::Output);

            if (captureOutcome.IsSuccess() && captureOutcomeDepth.IsSuccess())
            {
                AZStd::unique_lock<AZStd::mutex> lock(m_mutex);
                m_pendingFrames[time] = PendingFrames();
            }
            else
            {
                AZ_Error("SimpleLidar", false, "Failed to capture frame: %s", captureOutcome.GetError().m_errorMessage.c_str());
            }
        }
    }

    void SimpleLidar::PublishPointCloud(const PendingFrames& completedFrame)
    {
        // Get ROS2 timestamp and frame ID
        const auto simTimestamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        const auto frameId = GetNamespacedFrameID();

        const float width = static_cast<float>(completedFrame.m_viewsDataDepth.at(0).cols);
        const float height = static_cast<float>(completedFrame.m_viewsDataDepth.at(0).rows);

        // Pre-compute ray directions - estimate based on step sizes
        AZStd::vector<AZ::Vector3> rayDirections;
        AZStd::vector<uint16_t> rayRings;
        if (m_rayCount)
        {
            rayDirections.reserve(m_rayCount.value());
        }
        // Generate ray directions using configuration parameters
        const auto layerElevations = m_lidarConfiguration.GetLayerElevations();
        const float azimuthStep = m_lidarConfiguration.GetAzimuthStepRad();


        for (float azimuth = 0; azimuth < (2.0f * M_PI); azimuth += azimuthStep)
        {
            for (int ring = 0; ring < layerElevations.size(); ++ring)
            {
                const auto elevation = layerElevations[ring];
                AZ::Vector3 direction{ AZ::Cos(elevation) * AZ::Cos(azimuth),
                                       AZ::Cos(elevation) * AZ::Sin(azimuth),
                                       AZ::Sin(elevation) };
                rayDirections.emplace_back(direction.GetNormalized());
                rayRings.push_back(ring);
            }
        }

        m_rayCount = rayDirections.size();

        // First pass: collect valid points
        struct ValidPoint
        {
            AZ::Vector3 position;
            cv::Vec4b color;
            uint16_t ring;
        };

        // publish debug image if configured
        if (m_lidarConfiguration.m_publishDebugImages)
        {
            std::vector<cv::Mat> images;
            for (int i = 0; i < ViewCount; ++i)
            {
                images.push_back(completedFrame.m_viewsDataColor.at(i));
            }
            cv::Mat pointcloudImage;
            cv::hconcat(images, pointcloudImage);

            // draw rings

            for (int rayId = 0; rayId < rayDirections.size(); ++rayId)
            {
                const auto& direction = rayDirections[rayId];
                for (int viewId = 0; viewId < ViewCount; ++viewId)
                {
                    // Rotate direction into view space
                    const AZ::Vector3 localDirection = m_cameraToLidarCoordinate[viewId].GetInverse().TransformVector(direction);

                    // Project into image plane
                    const AZ::Vector3 uvw = m_cameraMatrix * localDirection.GetNormalized();
                    const float u = uvw.GetX() / uvw.GetZ();
                    const float v = uvw.GetY() / uvw.GetZ();

                    if (uvw.GetZ() > 0 && u >= 0 && u < width && v >= 0 && v < height)
                    {
                        const int ui = static_cast<int>(u);
                        const int vi = static_cast<int>(v);
                        cv::circle(pointcloudImage, cv::Point2f(ui + viewId * width, vi), 1, cv::Scalar(255, 0, 0, 128), -1);
                    }
                }
            }

            // publish
            sensor_msgs::msg::Image imageMessage;
            imageMessage.header.stamp = simTimestamp;
            imageMessage.header.frame_id = frameId.c_str();
            imageMessage.height = pointcloudImage.rows;
            imageMessage.width = pointcloudImage.cols;
            imageMessage.encoding = "rgba8";
            imageMessage.is_bigendian = false;
            imageMessage.step = static_cast<sensor_msgs::msg::Image::_step_type>(pointcloudImage.cols * sizeof(float));
            imageMessage.data.resize(pointcloudImage.rows * pointcloudImage.cols * sizeof(float));
            memcpy(imageMessage.data.data(), pointcloudImage.data, imageMessage.step * imageMessage.height);
            m_debugImagePublisher->publish(imageMessage);

        }


        AZStd::vector<ValidPoint> validPoints;
        validPoints.reserve(rayDirections.size());

        for (int rayId = 0; rayId < rayDirections.size(); ++rayId)
        {
            const auto& direction = rayDirections[rayId];
            const auto& ring = rayRings[rayId];
            for (int viewId = 0; viewId < ViewCount; ++viewId)
            {
                // Rotate direction into view space
                const AZ::Vector3 localDirection = m_cameraToLidarCoordinate[viewId].GetInverse().TransformVector(direction);

                // Project into image plane
                const AZ::Vector3 uvw = m_cameraMatrix * localDirection.GetNormalized();
                const float u = uvw.GetX() / uvw.GetZ();
                const float v = uvw.GetY() / uvw.GetZ();

                if (uvw.GetZ() > 0 && u >= 0 && u < width && v >= 0 && v < height)
                {
                    const int ui = static_cast<int>(u);
                    const int vi = static_cast<int>(v);
                    const cv::Vec4b& color = completedFrame.m_viewsDataColor.at(viewId).at<cv::Vec4b>(vi, ui);
                    //const float depthPlanar = completedFrame.m_viewsDataDepth.at(viewId).at<float>(vi, ui);
                    const float depthPlanar = sampleBilinear(completedFrame.m_viewsDataDepth.at(viewId), u, v);

                    if (depthPlanar > m_lidarConfiguration.m_minRange)
                    {
                        const float depthRange = PlanarDepthToRange(m_cameraMatrix, u, v, depthPlanar);

                        // Apply range limits from configuration
                        if (depthRange >= m_lidarConfiguration.m_minRange && depthRange <= m_lidarConfiguration.m_maxRange)
                        {
                            const AZ::Vector3 point = direction * depthRange;
                            validPoints.push_back({point, color, ring});

                            break; // Found hit for this ray, move to next ray
                        }
                    }
                }
            }
        }

        // Create point cloud message
        sensor_msgs::msg::PointCloud2 message;


        // add fields
        size_t currentDataOffset =0;
        message.fields.resize(3); // reserve space for x,y,z
        message.fields[0].name = "x";
        message.fields[0].datatype =PointcloudXYZTypeId;
        message.fields[0].offset = currentDataOffset;
        message.fields[0].count = 1;
        currentDataOffset += sizeof(PointcloudXYZType);

        message.fields[1].name = "y";
        message.fields[1].datatype = PointcloudXYZTypeId;
        message.fields[1].offset = currentDataOffset;
        message.fields[1].count = 1;
        currentDataOffset += sizeof(PointcloudXYZType);

        message.fields[2].name = "z";
        message.fields[2].datatype = PointcloudXYZTypeId;
        message.fields[2].count = 1;
        message.fields[2].offset = currentDataOffset;
        currentDataOffset += sizeof(PointcloudXYZType);


        if (m_lidarConfiguration.m_publishRGB)
        {
            message.fields.resize(message.fields.size() + 1);
            message.fields.back().name = "rgb";
            message.fields.back().offset = currentDataOffset;
            message.fields.back().datatype = PointcloudRGBTypeId;
            message.fields.back().count = 3; // RGB is 3 elements packed in float
            currentDataOffset += sizeof(PointcloudRGBType);
        }
        if (m_lidarConfiguration.m_publishRing)
        {
            message.fields.resize(message.fields.size() + 1);
            message.fields.back().name = m_lidarConfiguration.m_ringFieldName.c_str();
            message.fields.back().offset = currentDataOffset;
            message.fields.back().datatype = PointcloudRingTypeId;
            message.fields.back().count = 1;
            currentDataOffset += sizeof(PointcloudRingType);
        }

        if (m_lidarConfiguration.m_intensityFromRGB)
        {
            message.fields.resize(message.fields.size() + 1);
            message.fields.back().name = m_lidarConfiguration.m_intensity.c_str();
            message.fields.back().offset = currentDataOffset;
            message.fields.back().datatype = PointcloudIntensityTypeId;
            message.fields.back().count = 1;
            currentDataOffset += sizeof(PointcloudIntensityType);
        }

        sensor_msgs::PointCloud2Modifier modifier(message);

        message.point_step = currentDataOffset;
        message.row_step = currentDataOffset;
        message.width = validPoints.size();
        message.height = 1;

        message.header.stamp = simTimestamp;
        message.header.frame_id = frameId.c_str();
        message.is_dense = false;
        message.data.resize(validPoints.size() * message.point_step);

        PointcloudXYZIterator iter_x(message, "x");
        PointcloudXYZIterator iter_y(message, "y");
        PointcloudXYZIterator iter_z(message, "z");
        OptionalColorIterator iter_r, iter_g, iter_b;
        OptionalRingIterator iter_ring;
        OptionalIntensityIterator iter_intensity;

        if (m_lidarConfiguration.m_publishRGB)
        {

            iter_r = sensor_msgs::PointCloud2Iterator<uint8_t>(message, "r");
            iter_g = sensor_msgs::PointCloud2Iterator<uint8_t>(message, "g");
            iter_b = sensor_msgs::PointCloud2Iterator<uint8_t>(message, "b");
        }

        if (m_lidarConfiguration.m_publishRing)
        {
            const auto fieldName = m_lidarConfiguration.m_ringFieldName;
            iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(message, fieldName.c_str());
        }

        if (m_lidarConfiguration.m_intensityFromRGB)
        {
            const auto fieldName = m_lidarConfiguration.m_intensity;
            iter_intensity = sensor_msgs::PointCloud2Iterator<float>(message, fieldName.c_str());
        }

        // Fill point cloud data from cached valid points
        for (const auto& validPoint : validPoints)
        {
            *iter_x = validPoint.position.GetX();
            *iter_y = validPoint.position.GetY();
            *iter_z = validPoint.position.GetZ();

            if (iter_ring)
            {
                **iter_ring = validPoint.ring;
                ++(*iter_ring);
            }

            if (iter_intensity)
            {
                **iter_intensity = 0.299f * validPoint.color[2] + 0.587f * validPoint.color[1] + 0.114f * validPoint.color[0];
                ++(*iter_intensity);
            }

            if (iter_r && iter_g && iter_b)
            {
                **iter_r = validPoint.color[2]; // R (OpenCV is BGR)
                **iter_g = validPoint.color[1]; // G
                **iter_b = validPoint.color[0]; // B
                ++(*iter_r); ++(*iter_g); ++(*iter_b);
            }

            // Advance XYZ iterators
            ++iter_x; ++iter_y; ++iter_z;
        }

        // Publish the message
        m_pointCloudPublisher->publish(message);
    }

} // namespace SimpleLidarSensor