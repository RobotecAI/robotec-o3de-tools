/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include "GrassEditorSystemComponent.h"

#include <Grass/GrassTypeIds.h>

namespace Grass
{
    AZ_COMPONENT_IMPL(GrassEditorSystemComponent, "GrassEditorSystemComponent",
        GrassEditorSystemComponentTypeId, BaseSystemComponent);

    void GrassEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GrassEditorSystemComponent, GrassSystemComponent>()
                ->Version(0);
        }
    }

    GrassEditorSystemComponent::GrassEditorSystemComponent() = default;

    GrassEditorSystemComponent::~GrassEditorSystemComponent() = default;

    void GrassEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("GrassSystemEditorService"));
    }

    void GrassEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("GrassSystemEditorService"));
    }

    void GrassEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void GrassEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void GrassEditorSystemComponent::Activate()
    {
        GrassSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void GrassEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        GrassSystemComponent::Deactivate();
    }

} // namespace Grass
