/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{
    AZ_COMPONENT_IMPL(
        PointcloudEditorSystemComponent, "PointcloudEditorSystemComponent", PointcloudEditorSystemComponentTypeId, BaseSystemComponent);

    void PointcloudEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudEditorSystemComponent, PointcloudSystemComponent>()->Version(0);
        }
    }

    PointcloudEditorSystemComponent::PointcloudEditorSystemComponent() = default;

    PointcloudEditorSystemComponent::~PointcloudEditorSystemComponent() = default;

    void PointcloudEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("PointcloudSystemEditorService"));
    }

    void PointcloudEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("PointcloudSystemEditorService"));
    }

    void PointcloudEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void PointcloudEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void PointcloudEditorSystemComponent::Activate()
    {
        PointcloudSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void PointcloudEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        PointcloudSystemComponent::Deactivate();
    }

} // namespace Pointcloud
