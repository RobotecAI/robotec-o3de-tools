/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GrassSystemComponent.h"

#include <Grass/GrassTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

#include <Atom/RPI.Public/FeatureProcessorFactory.h>

#include <Render/GrassFeatureProcessor.h>

namespace Grass
{
    AZ_COMPONENT_IMPL(GrassSystemComponent, "GrassSystemComponent",
        GrassSystemComponentTypeId);

    void GrassSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GrassSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }

        GrassFeatureProcessor::Reflect(context);
    }

    void GrassSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GrassSystemService"));
    }

    void GrassSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GrassSystemService"));
    }

    void GrassSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("RPISystem"));
    }

    void GrassSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    GrassSystemComponent::GrassSystemComponent()
    {
        if (GrassInterface::Get() == nullptr)
        {
            GrassInterface::Register(this);
        }
    }

    GrassSystemComponent::~GrassSystemComponent()
    {
        if (GrassInterface::Get() == this)
        {
            GrassInterface::Unregister(this);
        }
    }

    void GrassSystemComponent::Init()
    {
    }

    void GrassSystemComponent::Activate()
    {
        GrassRequestBus::Handler::BusConnect();

    }

    void GrassSystemComponent::Deactivate()
    {
        GrassRequestBus::Handler::BusDisconnect();
    }

} // namespace Grass
