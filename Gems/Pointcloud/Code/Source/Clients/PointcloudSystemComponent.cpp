/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudSystemComponent.h"

#include <Pointcloud/PointcloudTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

#include <Atom/RPI.Public/FeatureProcessorFactory.h>

#include <Render/PointcloudFeatureProcessor.h>

namespace Pointcloud
{
    AZ_COMPONENT_IMPL(PointcloudSystemComponent, "PointcloudSystemComponent",
        PointcloudSystemComponentTypeId);

    void PointcloudSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }

        PointcloudFeatureProcessor::Reflect(context);
    }

    void PointcloudSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PointcloudSystemService"));
    }

    void PointcloudSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PointcloudSystemService"));
    }

    void PointcloudSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("RPISystem"));
    }

    void PointcloudSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    PointcloudSystemComponent::PointcloudSystemComponent()
    {
        if (PointcloudInterface::Get() == nullptr)
        {
            PointcloudInterface::Register(this);
        }
    }

    PointcloudSystemComponent::~PointcloudSystemComponent()
    {
        if (PointcloudInterface::Get() == this)
        {
            PointcloudInterface::Unregister(this);
        }
    }

    void PointcloudSystemComponent::Init()
    {
    }

    void PointcloudSystemComponent::Activate()
    {
        PointcloudRequestBus::Handler::BusConnect();

    }

    void PointcloudSystemComponent::Deactivate()
    {
        PointcloudRequestBus::Handler::BusDisconnect();
    }

} // namespace Pointcloud
