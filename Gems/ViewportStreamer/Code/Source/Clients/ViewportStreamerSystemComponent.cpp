
#include "ViewportStreamerSystemComponent.h"
#include "Utils/RegistryUtils.h"
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/View.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2/ROS2Bus.h>
#include <ViewportStreamer/ViewportStreamerTypeIds.h>
#include <rclcpp/qos.hpp>
#include <sstream>
#include <string>

namespace ViewportStreamer
{
    namespace
    {
        const AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, "rgba8" },
            { AZ::RHI::Format::B8G8R8A8_UNORM, "bgra8" },
        };

        const AZStd::unordered_map<AZ::RHI::Format, int> BitDepth{
            { AZ::RHI::Format::R8G8B8A8_UNORM, 4 * sizeof(uint8_t) },
            { AZ::RHI::Format::B8G8R8A8_UNORM, 4 * sizeof(uint8_t) },
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

        void RequestFrame(
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
                AZ_Warning("ViewportStreamerSystemComponent", false, captureOutcome.GetError().m_errorMessage.c_str());
            }
        }
    } // namespace

    AZ_COMPONENT_IMPL(ViewportStreamerSystemComponent, "ViewportStreamerSystemComponent", ViewportStreamerSystemComponentTypeId);

    void ViewportStreamerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ViewportStreamerSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ViewportStreamerSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ViewportStreamerService"));
    }

    void ViewportStreamerSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ViewportStreamerService"));
    }

    void ViewportStreamerSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    ViewportStreamerSystemComponent::ViewportStreamerSystemComponent()
    {
        m_imagePublisherTopic.m_type = "sensor_msgs::msg::Image";
        m_imagePublisherTopic.m_topic = "/viewport";
    }

    ViewportStreamerSystemComponent::~ViewportStreamerSystemComponent()
    {
    }

    void ViewportStreamerSystemComponent::Init()
    {
    }

    void ViewportStreamerSystemComponent::Activate()
    {
        m_frameName = RegistryUtilities::GetViewportStreamerFrameName();
        m_frequency = RegistryUtilities::GetViewportStreamerFrequency();

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (!ros2Node)
        {
            AZ_Error("ViewportStreamerSystemComponent", false, "ROS2 node is not available");
            return;
        }
        rclcpp::QoS publisherQoS = m_imagePublisherTopic.GetQoS();
        publisherQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);
        m_imagePublisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(m_imagePublisherTopic.m_topic.c_str(), publisherQoS);

        m_eventSourceAdapter.SetFrequency(m_frequency);
        m_adaptedEventHandler = decltype(m_adaptedEventHandler)(
            [this]([[maybe_unused]] auto&&... args)
            {
                FrequencyTick();
            });
        m_eventSourceAdapter.ConnectToAdaptedEvent(m_adaptedEventHandler);
        m_eventSourceAdapter.Start();
    }

    void ViewportStreamerSystemComponent::Deactivate()
    {
        m_eventSourceAdapter.Stop();
        m_adaptedEventHandler.Disconnect();
        m_imagePublisher.reset();
    }

    void ViewportStreamerSystemComponent::FrequencyTick()
    {
        auto atomViewportRequests = AZ::Interface<AZ::RPI::ViewportContextRequestsInterface>::Get();
        if (AZ::RPI::ViewportContextPtr viewportContext = atomViewportRequests->GetDefaultViewportContext())
        {
            if (auto pipeline = viewportContext->GetCurrentPipeline())
            {
                auto passHierarchy = GetPassHierarchyFromRenderPipeline(pipeline);

                std_msgs::msg::Header messageHeader;
                messageHeader.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
                messageHeader.frame_id = m_frameName.c_str();

                RequestMessagePublication(passHierarchy, messageHeader);
            }
        }
    }

    void ViewportStreamerSystemComponent::RequestMessagePublication(
        const AZStd::vector<AZStd::string>& passHierarchy, const std_msgs::msg::Header& header)
    {
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

} // namespace ViewportStreamer
