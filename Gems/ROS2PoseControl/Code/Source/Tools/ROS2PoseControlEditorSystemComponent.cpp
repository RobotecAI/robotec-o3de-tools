/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "ROS2PoseControlEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2PoseControl/ROS2PoseControlTypeIds.h>

namespace ROS2PoseControl
{
    AZ_COMPONENT_IMPL(
        ROS2PoseControlEditorSystemComponent,
        "ROS2PoseControlEditorSystemComponent",
        ROS2PoseControlEditorSystemComponentTypeId,
        BaseSystemComponent);

    void ROS2PoseControlEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2PoseControlEditorSystemComponent, ROS2PoseControlSystemComponent>()->Version(0);
        }
    }

    ROS2PoseControlEditorSystemComponent::ROS2PoseControlEditorSystemComponent() = default;

    ROS2PoseControlEditorSystemComponent::~ROS2PoseControlEditorSystemComponent() = default;

    void ROS2PoseControlEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2PoseControlEditorService"));
    }

    void ROS2PoseControlEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2PoseControlEditorService"));
    }

    void ROS2PoseControlEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ROS2PoseControlEditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ROS2PoseControlEditorSystemComponent::Activate()
    {
        ROS2PoseControlSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ROS2PoseControlEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ROS2PoseControlSystemComponent::Deactivate();
    }

} // namespace ROS2PoseControl
