/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "ROS2PoseControlModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <Clients/ROS2PoseControl.h>
#include <ROS2PoseControl/ROS2PoseControlTypeIds.h>

#include <Clients/ROS2PoseControlSystemComponent.h>

namespace ROS2PoseControl
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2PoseControlModuleInterface, "ROS2PoseControlModuleInterface", ROS2PoseControlModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2PoseControlModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2PoseControlModuleInterface, AZ::SystemAllocator);

    ROS2PoseControlModuleInterface::ROS2PoseControlModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(), { ROS2PoseControlSystemComponent::CreateDescriptor(), ROS2PoseControl::CreateDescriptor() });
    }

    AZ::ComponentTypeList ROS2PoseControlModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2PoseControlSystemComponent>(),
        };
    }
} // namespace ROS2PoseControl
