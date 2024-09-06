/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudComponent.h"
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace Pointcloud
{

    PointcloudComponent::PointcloudComponent(const PointcloudComponentConfig& config)
        : PointcloudComponentBase(config)
    {
    }

    void PointcloudComponent::Reflect(AZ::ReflectContext* context)
    {
        PointcloudComponentBase::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudComponent, PointcloudComponentBase>()->Version(1);
        }
    }

    void PointcloudComponent::Activate()
    {
        PointcloudComponentBase::Activate();
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void PointcloudComponent::Deactivate()
    {
        PointcloudComponentBase::Deactivate();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
    }

    void PointcloudComponent::OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world)
    {
        m_controller.OnTransformChanged(local, world);
    }
} // namespace Pointcloud
