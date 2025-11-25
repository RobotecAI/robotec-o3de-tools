/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include "RobotecGeoJSONSpawnerEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <RobotecGeoJSONSpawner/RobotecGeoJSONSpawnerTypeIds.h>

namespace RobotecGeoJSONSpawner
{
    AZ_COMPONENT_IMPL(
        RobotecGeoJSONSpawnerEditorSystemComponent,
        "RobotecGeoJSONSpawnerEditorSystemComponent",
        RobotecGeoJSONSpawnerEditorSystemComponentTypeId,
        BaseSystemComponent);

    void RobotecGeoJSONSpawnerEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotecGeoJSONSpawnerEditorSystemComponent, RobotecGeoJSONSpawnerSystemComponent>()->Version(0);
        }
    }

    void RobotecGeoJSONSpawnerEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerEditorService"));
    }

    void RobotecGeoJSONSpawnerEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("RobotecGeoJSONSpawnerEditorService"));
    }

    void RobotecGeoJSONSpawnerEditorSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void RobotecGeoJSONSpawnerEditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void RobotecGeoJSONSpawnerEditorSystemComponent::Activate()
    {
        RobotecGeoJSONSpawnerSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void RobotecGeoJSONSpawnerEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        RobotecGeoJSONSpawnerSystemComponent::Deactivate();
    }

} // namespace RobotecGeoJSONSpawner
