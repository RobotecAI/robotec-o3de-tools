#include "SimpleLidar.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>


#include <Atom/Feature/PostProcess/PostProcessFeatureProcessorInterface.h>
#include "AzCore/Math/Matrix4x4.h"

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/TransformBus.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <AzCore/Math/MatrixUtils.h>
#include <fstream>
namespace SimpleLidarSensor
{

    namespace
    {
        float GetAspectRatio(float width, float height)
        {
            return width / height;
        };

        AZ::Matrix3x3 MakeCameraIntrinsics(int width, int height, float verticalFieldOfViewDeg)
        {
            const auto w = static_cast<float>(width);
            const auto h = static_cast<float>(height);
            const float verticalFieldOfView = AZ::DegToRad(verticalFieldOfViewDeg);
            const float horizontalFoV = 2.0 * AZStd::atan(AZStd::tan(verticalFieldOfView / 2.0) * GetAspectRatio(width, height));
            const float focalLengthX = w / (2.0 * AZStd::tan(horizontalFoV / 2.0));
            const float focalLengthY = h / (2.0 * AZStd::tan(verticalFieldOfView / 2.0));
            return AZ::Matrix3x3::CreateFromRows({ focalLengthX, 0.f, w / 2.f }, { 0.f, focalLengthY, h / 2.f }, { 0.f, 0.f, 1.f });
        }

        AZ::Matrix4x4 MakeClipMatrix(int width, int height, float verticalFieldOfViewDeg, float nearDist, float farDist)
        {
            AZ::Matrix4x4 localViewToClipMatrix;
            AZ::MakePerspectiveFovMatrixRH(
                localViewToClipMatrix,
                AZ::DegToRad(verticalFieldOfViewDeg),
                GetAspectRatio(width, height),
                nearDist,
                farDist,
                true);
            return localViewToClipMatrix;
        }
        //! Returns a transformation matrix (rotation only) for given view index
        AZ::Transform GetViewTransform(int viewIndex, float horizontalFOV)
        {
            const float angle = AZ::DegToRad(viewIndex * horizontalFOV);
            const AZ::Quaternion localRot = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), -angle);
            return AZ::Transform::CreateFromQuaternion(localRot);
        }

        // Rotate whole camera camera optical (Z forward) to World (X forward)
        AZ::Transform GetCameraRigRotation()
        {
            AZ::Matrix3x3 matrix = AZ::Matrix3x3::CreateFromRows(
                AZ::Vector3::CreateAxisZ(),
                -AZ::Vector3::CreateAxisX(),
                -AZ::Vector3::CreateAxisY()
            );

            return AZ::Transform::CreateFromMatrix3x3(matrix);
        }

        float PlanarDepthToRange(const AZ::Matrix3x3& cameraMatrix, float u, float v, float planarDepth)
        {
            const float& fx = cameraMatrix.GetElement(0, 0);
            const float& fy = cameraMatrix.GetElement(1, 1);
            const float& cx = cameraMatrix.GetElement(0, 2);
            const float& cy = cameraMatrix.GetElement(1, 2);
            const float uc = u - cx;
            const float vc = v - cy;
            const float range = planarDepth * AZStd::sqrt((uc*uc) / (fx * fx) + (vc * vc) / (fy * fy) + 1.0f);
            return range;
        }


    }

    // Reflect for serialization and scripting
    void SimpleLidar::Reflect(AZ::ReflectContext* context)
    {
        if (auto serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SimpleLidar, AZ::Component>()
            ->Version(1)
            ->Field("value", &SimpleLidar::m_value);


            if (auto ec = serialize->GetEditContext())
            {
                ec->Class<SimpleLidar>("SimpleLidar", "SimpleLidar using graphics pipeline")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"));
            }
        }

    }

    void SimpleLidar::Activate()
    {
        AZ::TickBus::Handler::BusConnect();

        m_pipelines.resize(ViewCount);
        m_view.resize(ViewCount);
        m_passHierarchies.resize(ViewCount);
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        const int width = 640;
        const int height = 640;
        const float VerticalFOV = HorizontalFOV / GetAspectRatio(width, height);

        const AZ::Matrix4x4 localViewToClipMatrix = MakeClipMatrix(
           width,width, HorizontalFOV, 0.1f, 100.0f);
        m_cameraMatrix = MakeCameraIntrinsics(width, height, VerticalFOV);

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
            pipelineDesc.m_rootPassTemplate = "PipelineRenderToTextureROSColor";
            pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
            pipeline->RemoveFromRenderTick();
            if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(pipeline->GetRootPass().get()))
            {
                renderToTexturePass->ResizeOutput(
                   width,height);
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

            const auto viewTransform = GetViewTransform(i, HorizontalFOV)  * GetCameraRigRotation();
             m_cameraToLidarCoordinate.push_back(viewTransform);
        }
    }


    void SimpleLidar::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
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


    void SimpleLidar::FrameComplete(const PendingFrames& completedFrame)
    {
        static int frameId = 0;
        frameId++;
        if (frameId == 10)
        {
            std::vector<cv::Mat> frames;
            for (int i = 0; i < ViewCount; ++i)
            {
                frames.push_back(completedFrame.m_viewsDataColor.at(i));
            }
            cv::Mat combined;
            cv::hconcat(frames, combined);
            cv::imwrite("/tmp/lidar.png", combined);


            std::ofstream pc ("/tmp/lidar.txt");
            // loop over 360 degree

            AZ_Assert(!completedFrame.m_viewsDataDepth.empty(), "it should not be empty");
            if (completedFrame.m_viewsDataDepth.empty())
            {
                return;
            }

            const float width = static_cast<float>(completedFrame.m_viewsDataDepth.at(0).cols);
            const float height = static_cast<float>(completedFrame.m_viewsDataDepth.at(0).rows);

            // test view 0
            std::ofstream pc_view0 ("/tmp/lidar_view0.txt");
            const float fx = m_cameraMatrix.GetElement(0,0);
            const float fy = m_cameraMatrix.GetElement(1,1);
            const float cx = m_cameraMatrix.GetElement(0,2);
            const float cy = m_cameraMatrix.GetElement(1,2);
            for (int i=0; i < ViewCount; ++i)
            {
                const auto & colorMat = completedFrame.m_viewsDataColor.at(i);
                const auto & depthMat = completedFrame.m_viewsDataDepth.at(i);
                const auto & transform = m_cameraToLidarCoordinate.at(i);


                for (int u =0; u < width; ++u)
                {
                    for (int v =0; v < height; ++v)
                    {
                        const cv::Vec4b& color = colorMat.at<cv::Vec4b>(v, u);
                        const float depth = depthMat.at<float>(v, u);
                        if (depth > 0.1)
                        {

                            const float x = (static_cast<float>(u) - cx) * depth / fx;
                            const float y = (static_cast<float>(v) - cy) * depth / fy;
                            const float z = depth;
                            const AZ::Vector3 point = transform.TransformPoint(AZ::Vector3(x,y,z));


                            pc_view0 << (float) point.GetX()<< " " << (float) point.GetY() << " " << (float) point.GetZ() << " "
                            << (int)color[0] << " " << (int) color[1] <<" " << (int)color[2] << std::endl;
                        }
                    }
                }
            }


            for (float azimuth = 0; azimuth < (2.0*M_PI); azimuth += (2.0*M_PI)/1024)
            {
                for (float elevation = AZ::DegToRad(-180); elevation < AZ::DegToRad(180); elevation += AZ::DegToRad(0.1))
                {
                    AZ::Vector3 direction { AZ::Cos(elevation) * AZ::Cos(azimuth), AZ::Cos(elevation) * AZ::Sin(azimuth), AZ::Sin(elevation)  };
                    direction = direction.GetNormalized();
                    for (int viewId = 0; viewId < ViewCount; ++viewId)
                    {
                        //rotate direction into view space
                        const AZ::Vector3 localDirection = m_cameraToLidarCoordinate[viewId].GetInverse().TransformPoint(direction);

                        // project into image plane
                        const AZ::Vector3 uvw = m_cameraMatrix *  localDirection.GetNormalized();
                        const float u = uvw.GetX() / uvw.GetZ();
                        const float v = uvw.GetY() / uvw.GetZ();
                        if (uvw.GetZ() > 0 && u >= 0 && u < width && v >= 0 && v < height)
                        {
                            // we have a hit
                            const int ui = static_cast<int>(u);
                            const int vi = static_cast<int>(v);
                            const cv::Vec4b& color = completedFrame.m_viewsDataColor.at(viewId).at<cv::Vec4b>(vi, ui);
                            const float depthPlanar = completedFrame.m_viewsDataDepth.at(viewId).at<float>(vi, ui);
                            const float depthRange = PlanarDepthToRange(m_cameraMatrix, u, v, depthPlanar);

                            if (depthPlanar > 0.1)
                            {
                                const AZ::Vector3 point = direction * depthRange;
                                pc << (float) point.GetX() << " " << (float) point.GetY() << " " << (float) point.GetZ() << " "
                                << (int)color[0] << " " << (int) color[1] <<" " << (int)color[2] << std::endl;

                                 break; // next ray
                            }
                        }
                    }
                }
            }
        }
    }

    void SimpleLidar::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {

        //! Coordinate system conversion from O3DE/Atom to OpenCV (Z forward, X right, -Y down)
        const AZ::Transform AtomToCv{ AZ::Transform::CreateFromQuaternion(
           AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };

        const auto EntityPose = GetEntity()->GetTransform()->GetWorldTM();
        const auto EntityPoseNoScaling = AZ::Transform::CreateFromQuaternionAndTranslation(EntityPose.GetRotation(), EntityPose.GetTranslation());


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

            const AZ::Transform cameraPose = (EntityPoseNoScaling * viewTransform * AtomToCv  ).GetInverse();
            view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromTransform(cameraPose));
            pipeline->AddToRenderTickOnce();
            AZ::Render::FrameCaptureOutcome captureOutcome;

            auto callback = [this,  viewIndex = i, ts = time.Get()](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {

                const AZStd::unordered_map<AZ::RHI::Format, int> FormatToCvFormat = {
                    {AZ::RHI::Format::R8G8B8A8_UNORM, CV_8UC4},
                    {AZ::RHI::Format::R32_FLOAT, CV_32F}
                };

                if (result.m_state == AZ::RPI::AttachmentReadback::ReadbackState::Success)
                {
                    AZStd::unique_lock<AZStd::mutex> lock(m_mutex);
                    if (m_pendingFrames.contains(ts))
                    {
                        auto& pendingFrame = m_pendingFrames[ts];

                        // convert to cv::Mat
                        const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                        const auto format = descriptor.m_format;

                        auto formatIt  = FormatToCvFormat.find(format);
                        AZ_Assert(formatIt != FormatToCvFormat.end() , "Unexpected format in result %u", static_cast<uint32_t>(format));
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
                                pendingFrame.ReportColorFrameCaptured(viewIndex,  frame);
                            }

                            if (pendingFrame.IsComplete())
                            {
                                FrameComplete(pendingFrame);
                                m_pendingFrames.erase(ts);
                            }
                        }
                    }
                }
                else
                {
                    AZ_Error("SimpleLidar", false, "Capture %d Failed", viewIndex);
                }
            };
            AZ::Render::FrameCaptureRequestBus::BroadcastResult(
                    captureOutcome, &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,callback, passHierarchy, AZStd::string("Output"),
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
                m_pendingFrames[time.Get()] = PendingFrames();
            }
            else
            {
                AZ_Error("SimpleLidar", false, "Failed to capture frame: %s", captureOutcome.GetError().m_errorMessage.c_str());
            }
        }
    }



}