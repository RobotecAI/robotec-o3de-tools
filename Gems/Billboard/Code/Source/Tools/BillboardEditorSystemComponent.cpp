/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include "BillboardEditorSystemComponent.h"

#include <Billboard/BillboardTypeIds.h>

namespace Billboard
{
    AZ_COMPONENT_IMPL(BillboardEditorSystemComponent, "BillboardEditorSystemComponent",
        BillboardEditorSystemComponentTypeId, BaseSystemComponent);

    void BillboardEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<BillboardEditorSystemComponent, BillboardSystemComponent>()
                ->Version(0);
        }
    }

    BillboardEditorSystemComponent::BillboardEditorSystemComponent() = default;

    BillboardEditorSystemComponent::~BillboardEditorSystemComponent() = default;

    void BillboardEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("BillboardSystemEditorService"));
    }

    void BillboardEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("BillboardSystemEditorService"));
    }

    void BillboardEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void BillboardEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void BillboardEditorSystemComponent::Activate()
    {
        BillboardSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void BillboardEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        BillboardSystemComponent::Deactivate();
    }

} // namespace Billboard
