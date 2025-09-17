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
namespace SimpleLidarSensor
{


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
        for (int i = 0; i < ViewCount; ++i)
        {
            auto& pipeline = m_pipelines[i];
            auto& view = m_view[i];
            auto& passHierarchy = m_passHierarchies[i];
            const AZ::Name viewName = AZ::Name("MainCamera");
            view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);

            AZ::Matrix4x4 localViewToClipMatrix;
            AZ::MakePerspectiveFovMatrixRH(
            localViewToClipMatrix,
                AZ::DegToRad(120.0f), // fov
                640.0f / 480.0f,    // aspect ratio
                0.1f,               // near clip
                100.0f, true);           // far clip
            view->SetViewToClipMatrix(localViewToClipMatrix);

            const auto pipelineName = AZStd::string::format("SimpleLidarCamera%s_%d", GetEntityId().ToString().c_str(), i);
            AZ::RPI::RenderPipelineDescriptor pipelineDesc;
            pipelineDesc.m_mainViewTagName = "MainCamera";

            pipelineDesc.m_allowModification = false;
            pipelineDesc.m_name = pipelineName;
            pipelineDesc.m_renderSettings.m_multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();
            pipelineDesc.m_rootPassTemplate = "PipelineRenderToTextureROSColor";
            pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
            //m_pipeline->RemoveFromRenderTick();
            if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(pipeline->GetRootPass().get()))
            {
                renderToTexturePass->ResizeOutput(
                   640,480);
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
        // required.push_back(AZ_CRC("SomeOtherService"));
    }


    void SimpleLidar::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {

        const AZ::Transform AtomToRos{ AZ::Transform::CreateFromQuaternion(
           AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };
        const auto EntityPose = GetEntity()->GetTransform()->GetWorldTM();
        const auto EntityPoseNoScaling = AZ::Transform::CreateFromQuaternionAndTranslation(EntityPose.GetRotation(), EntityPose.GetTranslation());
        for (int i = 0; i < ViewCount; ++i)
        {
            auto& pipeline = m_pipelines[i];
            auto& view = m_view[i];
            const auto& passHierarchy = m_passHierarchies[i];

            // rotate around Y axis
            const float angle = AZ::DegToRad(i * HorizontalFOV);
            const AZ::Quaternion localRot = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisY(), angle);
            const AZ::Transform cameraPose = (EntityPoseNoScaling * AtomToRos *  AZ::Transform::CreateFromQuaternion(localRot) ).GetInverse();
            view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromTransform(cameraPose));
            pipeline->AddToRenderTickOnce();
            AZStd::string fileName = AZStd::string::format("/tmp/SimpleLidar_%d.png", i);
            AZ::Render::FrameCaptureOutcome captureOutcome;
                AZ::Render::FrameCaptureRequestBus::BroadcastResult(
                        captureOutcome, &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachment, fileName, passHierarchy, AZStd::string("Output"),
                        AZ::RPI::PassAttachmentReadbackOption::Output);

            if (!captureOutcome.IsSuccess())
            {
                AZ_Error("SimpleLidar", false, "Failed to capture frame: %s", captureOutcome.GetError().m_errorMessage.c_str());
            }
        }


        // for (float f = 0.0f; f < 3.0f; f += 0.1f)
        // {
        //     const AZ::Transform AtomToRos{ AZ::Transform::CreateFromQuaternion(
        //        AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };
        //
        //      auto pose = GetEntity()->GetTransform()->GetWorldTM();
        //     pose.SetTranslation(pose.GetTranslation() + AZ::Vector3(f, 0.0f, 0.0f));
        //     const AZ::Transform cameraPoseNoScaling =
        //     AZ::Transform::CreateFromQuaternionAndTranslation(pose.GetRotation(), pose.GetTranslation());
        //     const AZ::Transform inverse = (cameraPoseNoScaling * AtomToRos).GetInverse();
        //     m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(), inverse.GetTranslation()));
        //     m_pipeline->AddToRenderTickOnce();
        //
        //     AZStd::string fileName = AZStd::string::format("/tmp/SimpleLidar_%f.png", f);
        //     AZ::Render::FrameCaptureOutcome captureOutcome;
        //     AZ::Render::FrameCaptureRequestBus::BroadcastResult(
        //         captureOutcome, &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachment, fileName, m_passHierarchy, AZStd::string("Output"),
        //         AZ::RPI::PassAttachmentReadbackOption::Output);
        //
        //     AZ_Printf("SimpleLidar", "Capture %s", captureOutcome.IsSuccess() ? "Success" : "Failed");
        //     if (!captureOutcome.IsSuccess())
        //     {
        //         AZ_Error("SimpleLidar", false, "Failed to capture frame: %s", captureOutcome.GetError().m_errorMessage.c_str());
        //     }
        //     break;
        // }


    }



}