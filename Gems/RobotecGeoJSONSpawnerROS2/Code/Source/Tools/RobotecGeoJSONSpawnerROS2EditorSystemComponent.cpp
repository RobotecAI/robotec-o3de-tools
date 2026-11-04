/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "RobotecGeoJSONSpawnerROS2EditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <RobotecGeoJSONSpawnerROS2/RobotecGeoJSONSpawnerROS2TypeIds.h>

namespace RobotecGeoJSONSpawnerROS2
{
    AZ_COMPONENT_IMPL(
        RobotecGeoJSONSpawnerROS2EditorSystemComponent,
        "RobotecGeoJSONSpawnerROS2EditorSystemComponent",
        RobotecGeoJSONSpawnerROS2EditorSystemComponentTypeId,
        BaseSystemComponent);

    void RobotecGeoJSONSpawnerROS2EditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecGeoJSONSpawnerROS2EditorSystemComponent, RobotecGeoJSONSpawnerROS2SystemComponent>()->Version(0);
        }
    }

    RobotecGeoJSONSpawnerROS2EditorSystemComponent::RobotecGeoJSONSpawnerROS2EditorSystemComponent() = default;

    RobotecGeoJSONSpawnerROS2EditorSystemComponent::~RobotecGeoJSONSpawnerROS2EditorSystemComponent() = default;

    void RobotecGeoJSONSpawnerROS2EditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerROS2EditorService"));
    }

    void RobotecGeoJSONSpawnerROS2EditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerROS2EditorService"));
    }

    void RobotecGeoJSONSpawnerROS2EditorSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void RobotecGeoJSONSpawnerROS2EditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void RobotecGeoJSONSpawnerROS2EditorSystemComponent::Activate()
    {
        RobotecGeoJSONSpawnerROS2SystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void RobotecGeoJSONSpawnerROS2EditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        RobotecGeoJSONSpawnerROS2SystemComponent::Deactivate();
    }

} // namespace RobotecGeoJSONSpawnerROS2
