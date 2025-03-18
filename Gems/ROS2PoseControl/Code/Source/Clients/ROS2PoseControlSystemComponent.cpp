/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "ROS2PoseControlSystemComponent.h"

#include <ROS2PoseControl/ROS2PoseControlTypeIds.h>

#include "ROS2PoseControlConfiguration.h"
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2PoseControl
{
    AZ_COMPONENT_IMPL(ROS2PoseControlSystemComponent, "ROS2PoseControlSystemComponent", ROS2PoseControlSystemComponentTypeId);

    void ROS2PoseControlSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2PoseControlConfiguration::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2PoseControlSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ROS2PoseControlSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2PoseControlService"));
    }

    void ROS2PoseControlSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2PoseControlService"));
    }

    void ROS2PoseControlSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2PoseControlSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2PoseControlSystemComponent::ROS2PoseControlSystemComponent()
    {
    }

    ROS2PoseControlSystemComponent::~ROS2PoseControlSystemComponent()
    {
    }

    void ROS2PoseControlSystemComponent::Init()
    {
    }

    void ROS2PoseControlSystemComponent::Activate()
    {
    }

    void ROS2PoseControlSystemComponent::Deactivate()
    {
    }

} // namespace ROS2PoseControl
