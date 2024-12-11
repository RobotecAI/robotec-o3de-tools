
#include "ViewportStreamerComponent.h"

#include <ViewportStreamer/ViewportStreamerTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/View.h>
#include <Atom/RPI.Public/ViewportContextBus.h>

#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2/ROS2Bus.h>
#include <sstream>
#include <string>

namespace ViewportStreamer
{
    namespace
    {
        const AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, "rgba8" },
            { AZ::RHI::Format::B8G8R8A8_UNORM, "bgra8" },
            { AZ::RHI::Format::R32_FLOAT, "32FC1" },
        };

        const AZStd::unordered_map<AZ::RHI::Format, int> BitDepth{
            { AZ::RHI::Format::R8G8B8A8_UNORM, 4 * sizeof(uint8_t) },
            { AZ::RHI::Format::B8G8R8A8_UNORM, 4 * sizeof(uint8_t) },
            { AZ::RHI::Format::R32_FLOAT, sizeof(float) },
        };

        sensor_msgs::msg::Image CreateImageMessageFromReadBackResult(
            const AZ::EntityId& entityId, const AZ::RPI::AttachmentReadback::ReadbackResult& result, const std_msgs::msg::Header& header)
        {
            const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
            const auto format = descriptor.m_format;
            AZ_Assert(FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
            sensor_msgs::msg::Image imageMessage;
            imageMessage.encoding = FormatMappings.at(format);
            imageMessage.width = descriptor.m_size.m_width;
            imageMessage.height = descriptor.m_size.m_height;
            imageMessage.step = imageMessage.width * BitDepth.at(format);
            imageMessage.data =
                std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
            imageMessage.header = header;
            ROS2::CameraPostProcessingRequestBus::Event(entityId, &ROS2::CameraPostProcessingRequests::ApplyPostProcessing, imageMessage);
            return imageMessage;
        }

        AZStd::vector<AZStd::string> GetPassHierarchyFromRenderPipeline(AZ::RPI::RenderPipelinePtr pipeline)
        {
            AZStd::vector<AZStd::string> passHierarchy;
            auto pipelineHierarchy = pipeline->GetRootPass()->GetPathName().GetCStr();
            std::string substr;
            std::istringstream pipelineStr(pipelineHierarchy);

            while (std::getline(pipelineStr, substr, '.'))
            {
                passHierarchy.push_back(AZStd::string(substr.c_str()));
            }
            passHierarchy.push_back("CopyToSwapChain");

            return passHierarchy;
        }

    } // namespace

    AZ_COMPONENT_IMPL(ViewportStreamerComponent, "ViewportStreamerComponent", ViewportStreamerComponentTypeId);

    void ViewportStreamerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ViewportStreamerComponent, AZ::Component>()
                ->Version(0)
                ->Field("TopicConfiguration", &ViewportStreamerComponent::m_imagePublisherTopic)
                ->Field("SensorConfiguration", &ViewportStreamerComponent::m_sensorConfiguration);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<ViewportStreamerComponent>(
                        "Viewport Streamer", "Provide data streaming from current viewport, via ROS2 publisher.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "RoSi")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ViewportStreamerComponent::m_imagePublisherTopic,
                        "Viewport Topic Configuration",
                        "Topic for viewport image data publisher")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ViewportStreamerComponent::m_sensorConfiguration,
                        "Sensor Configuration",
                        "Viewport Streamer sensor configuration");
            }
        }
    }

    void ViewportStreamerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ViewportStreamerService"));
    }

    void ViewportStreamerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ViewportStreamerService"));
    }

    void ViewportStreamerComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    ViewportStreamerComponent::ViewportStreamerComponent()
    {
        m_imagePublisherTopic.m_type = "sensor_msgs::msg::Image";
        m_imagePublisherTopic.m_topic = "/viewport";
    }

    ViewportStreamerComponent::~ViewportStreamerComponent()
    {
    }

    void ViewportStreamerComponent::Init()
    {
    }

    void ViewportStreamerComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();
        m_frameName = AZ::Uuid::CreateNull().ToString<AZStd::string>();

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "Entity has no ROS2 Node");
        m_imagePublisher =
            ros2Node->create_publisher<sensor_msgs::msg::Image>(m_imagePublisherTopic.m_topic.c_str(), m_imagePublisherTopic.GetQoS());

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                FrequencyTick();
            });
    }

    void ViewportStreamerComponent::Deactivate()
    {
        ROS2SensorComponentBase::Deactivate();
    }

    void ViewportStreamerComponent::FrequencyTick()
    {
        auto atomViewportRequests = AZ::Interface<AZ::RPI::ViewportContextRequestsInterface>::Get();
        AZ::RPI::ViewportContextPtr viewportContext = atomViewportRequests->GetDefaultViewportContext();
        auto pipeline = viewportContext->GetCurrentPipeline();

        auto passHierarchy = GetPassHierarchyFromRenderPipeline(pipeline);

        std_msgs::msg::Header messageHeader;
        messageHeader.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        messageHeader.frame_id = m_frameName.c_str();

        RequestMessagePublication(passHierarchy, messageHeader);
    }

    void ViewportStreamerComponent::RequestMessagePublication(
        const AZStd::vector<AZStd::string>& passHierarchy, const std_msgs::msg::Header& header)
    {
        if (!m_imagePublisher)
        {
            AZ_Error("ViewportStreamerComponent", false, "Missing publisher for the ViewportStreamer sensor");
            return;
        }

        auto imageReadyCallback = [this, header](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
        {
            if (result.m_state != AZ::RPI::AttachmentReadback::ReadbackState::Success)
            {
                return;
            }

            auto imageMessage = CreateImageMessageFromReadBackResult(GetEntityId(), result, header);
            m_imagePublisher->publish(imageMessage);
        };
        RequestFrame(passHierarchy, imageReadyCallback);
    }

    void ViewportStreamerComponent::RequestFrame(
        const AZStd::vector<AZStd::string>& passHierarchy,
        AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback)
    {
        AZ::Render::FrameCaptureOutcome captureOutcome;
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            captureOutcome,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            callback,
            passHierarchy,
            AZStd::string("Output"),
            AZ::RPI::PassAttachmentReadbackOption::Output);

        if (!captureOutcome.IsSuccess())
        {
            AZ_Warning("ViewportStreamerComponent", false, captureOutcome.GetError().m_errorMessage.c_str());
        }
    }

} // namespace ViewportStreamer
