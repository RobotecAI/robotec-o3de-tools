/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "BillboardSystemComponent.h"

#include <Billboard/BillboardTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

#include <Atom/RPI.Public/FeatureProcessorFactory.h>

#include <Render/BillboardFeatureProcessor.h>

namespace Billboard
{
    AZ_COMPONENT_IMPL(BillboardSystemComponent, "BillboardSystemComponent",
        BillboardSystemComponentTypeId);

    void BillboardSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<BillboardSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }

        BillboardFeatureProcessor::Reflect(context);
    }

    void BillboardSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("BillboardSystemService"));
    }

    void BillboardSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("BillboardSystemService"));
    }

    void BillboardSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("RPISystem"));
    }

    void BillboardSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    BillboardSystemComponent::BillboardSystemComponent()
    {
        if (BillboardInterface::Get() == nullptr)
        {
            BillboardInterface::Register(this);
        }
    }

    BillboardSystemComponent::~BillboardSystemComponent()
    {
        if (BillboardInterface::Get() == this)
        {
            BillboardInterface::Unregister(this);
        }
    }

    void BillboardSystemComponent::Init()
    {
    }

    void BillboardSystemComponent::Activate()
    {
        BillboardRequestBus::Handler::BusConnect();

    }

    void BillboardSystemComponent::Deactivate()
    {
        BillboardRequestBus::Handler::BusDisconnect();
    }

} // namespace Billboard
