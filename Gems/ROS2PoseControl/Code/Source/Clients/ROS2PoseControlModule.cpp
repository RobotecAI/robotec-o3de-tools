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
#include <ROS2PoseControlModuleInterface.h>

namespace ROS2PoseControl
{
    class ROS2PoseControlModule : public ROS2PoseControlModuleInterface
    {
    public:
        AZ_RTTI(ROS2PoseControlModule, ROS2PoseControlModuleTypeId, ROS2PoseControlModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2PoseControlModule, AZ::SystemAllocator);
    };
} // namespace ROS2PoseControl

AZ_DECLARE_MODULE_CLASS(Gem_ROS2PoseControl, ROS2PoseControl::ROS2PoseControlModule)
