/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include <ROS2PoseControl/ROS2PoseControlConfiguration.h>

#include <AzCore/Serialization/EditContext.h>

namespace ROS2PoseControl
{
    AZ::Crc32 ROS2PoseControlConfiguration::isTrackingModeTF2Visibility() const
    {
        return m_tracking_mode == TrackingMode::TF2 ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 ROS2PoseControlConfiguration::isTrackingModePoseMessagesVisibility() const
    {
        return m_tracking_mode == TrackingMode::PoseMessages ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 ROS2PoseControlConfiguration::isGroudOffsetVisible() const
    {
        return m_clampToGround ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 ROS2PoseControlConfiguration::isUseTagOffset() const
    {
        return m_useTagOffset ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    void ROS2PoseControlConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2PoseControlConfiguration>()
                ->Version(1)
                ->Field("m_tracking_mode", &ROS2PoseControlConfiguration::m_tracking_mode)
                ->Field("m_poseTopicConfiguration", &ROS2PoseControlConfiguration::m_poseTopicConfiguration)
                ->Field("m_targetFrame", &ROS2PoseControlConfiguration::m_targetFrame)
                ->Field("m_referenceFrame", &ROS2PoseControlConfiguration::m_referenceFrame)
                ->Field("lockZAxis", &ROS2PoseControlConfiguration::m_lockZAxis)
                ->Field("useTagOffset", &ROS2PoseControlConfiguration::m_useTagOffset)
                ->Field("startOffsetTag", &ROS2PoseControlConfiguration::m_startOffsetTag)
                ->Field("m_clampToGround", &ROS2PoseControlConfiguration::m_clampToGround)
                ->Field("m_groundOffset", &ROS2PoseControlConfiguration::m_groundOffset)
                ->Field("useWGS", &ROS2PoseControlConfiguration::m_useWGS);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2PoseControlConfiguration>("ROS2PoseControlConfiguration", "Sub configuration for ROS2PoseControl component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2PoseControlConfiguration::m_tracking_mode,
                        "Tracking Mode",
                        "Selects tracking mode")
                    ->EnumAttribute(TrackingMode::PoseMessages, "Pose Messages")
                    ->EnumAttribute(TrackingMode::TF2, "TF2")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControlConfiguration::m_useWGS,
                        "Enable support for WGS84",
                        "Enable support for WGS84")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2PoseControlConfiguration::isTrackingModePoseMessagesVisibility)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControlConfiguration::m_poseTopicConfiguration,
                        "Topic for control message",
                        "Configuration for ROS2 topic to receive control messages to")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2PoseControlConfiguration::isTrackingModePoseMessagesVisibility)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControlConfiguration::m_targetFrame,
                        "Target Frame",
                        "Frame to track eg. base_link")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2PoseControlConfiguration::isTrackingModeTF2Visibility)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControlConfiguration::m_referenceFrame,
                        "Reference Frame",
                        "Reference frame eg. map")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2PoseControlConfiguration::isTrackingModeTF2Visibility)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2PoseControlConfiguration::m_lockZAxis, "Lock Z Axis", "Lock Z axis")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControlConfiguration::m_useTagOffset,
                        "Use Tag Offset",
                        "Use a tag that will be used to set the start offset for the entity.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControlConfiguration::m_startOffsetTag,
                        "Start Offset Tag",
                        "Tag that will be used to set the start offset for the entity.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2PoseControlConfiguration::isUseTagOffset)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2PoseControlConfiguration::m_clampToGround, "Clamp to Ground", "Clamp to ground")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2PoseControlConfiguration::m_groundOffset,
                        "Ground Offset",
                        "Offset from the ground")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2PoseControlConfiguration::isGroudOffsetVisible);
            }
        }
    };
} // namespace ROS2PoseControl
