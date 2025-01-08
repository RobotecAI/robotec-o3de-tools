/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "GeoJSONSpawnerROS2EditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <GeoJSONSpawnerROS2/GeoJSONSpawnerROS2TypeIds.h>

namespace GeoJSONSpawnerROS2
{
    AZ_COMPONENT_IMPL(
        GeoJSONSpawnerROS2EditorSystemComponent,
        "GeoJSONSpawnerROS2EditorSystemComponent",
        GeoJSONSpawnerROS2EditorSystemComponentTypeId,
        BaseSystemComponent);

    void GeoJSONSpawnerROS2EditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoJSONSpawnerROS2EditorSystemComponent, GeoJSONSpawnerROS2SystemComponent>()->Version(0);
        }
    }

    GeoJSONSpawnerROS2EditorSystemComponent::GeoJSONSpawnerROS2EditorSystemComponent() = default;

    GeoJSONSpawnerROS2EditorSystemComponent::~GeoJSONSpawnerROS2EditorSystemComponent() = default;

    void GeoJSONSpawnerROS2EditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("GeoJSONSpawnerROS2EditorService"));
    }

    void GeoJSONSpawnerROS2EditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("GeoJSONSpawnerROS2EditorService"));
    }

    void GeoJSONSpawnerROS2EditorSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void GeoJSONSpawnerROS2EditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void GeoJSONSpawnerROS2EditorSystemComponent::Activate()
    {
        GeoJSONSpawnerROS2SystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void GeoJSONSpawnerROS2EditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        GeoJSONSpawnerROS2SystemComponent::Deactivate();
    }

} // namespace GeoJSONSpawnerROS2
