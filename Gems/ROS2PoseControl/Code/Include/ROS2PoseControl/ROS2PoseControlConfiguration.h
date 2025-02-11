
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Communication/TopicConfiguration.h>

namespace ROS2PoseControl
{
    //! A structure for a single ROS2 topic, a part of publisher or subscriber configuration.
    struct ROS2PoseControlConfiguration
    {
    public:
        AZ_TYPE_INFO(ROS2PoseControlConfiguration, "{4f7686d5-f44e-43a6-a1b8-f404065c6430}");

        static void Reflect(AZ::ReflectContext* context);

        AZ::Crc32 isTrackingModeTF2Visibility() const;

        AZ::Crc32 isTrackingModePoseMessagesVisibility() const;

        AZ::Crc32 isGroudOffsetVisible() const;

        AZ::Crc32 isUseTagOffset() const;

        enum class TrackingMode
        {
            PoseMessages,
            TF2
        };

        TrackingMode m_tracking_mode = TrackingMode::PoseMessages;
        ROS2::TopicConfiguration m_poseTopicConfiguration;

        AZStd::string m_targetFrame;
        AZStd::string m_referenceFrame;

        bool m_lockZAxis = true;

        bool m_clampToGround = false;
        float m_groundOffset = 0.0f;

        bool m_useTagOffset = false;
        AZStd::string m_startOffsetTag;

        bool m_useWGS = false;

        bool m_enablePhysics{ true };
        bool m_isKinematic{ false };
    };
} // namespace ROS2PoseControl
