/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudComponent.h"
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Pointcloud/PointcloudTypeIds.h>
#include <Render/PointcloudFeatureProcessor.h>

namespace Pointcloud
{
    PointcloudComponent::PointcloudComponent(const AZ::Data::Asset<PointcloudAsset>& pointcloudAsset, const float pointSize)
        : m_pointcloudAsset(pointcloudAsset)
        , m_pointSize(pointSize)
    {
    }

    void PointcloudComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudComponent, AZ::Component>()
                ->Version(0)
                ->Field("PointcloudAsset", &PointcloudComponent::m_pointcloudAsset)
                ->Field("PointSize", &PointcloudComponent::m_pointSize);
        }
    }

    void PointcloudComponent::Activate()
    {
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                m_scene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
                if (m_scene && m_pointcloudAsset)
                {
                    m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();
                    AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
                    m_pointcloudHandle = m_featureProcessor->AcquirePointcloudFromAsset(m_pointcloudAsset);
                    m_featureProcessor->SetTransform(m_pointcloudHandle, m_entity->GetTransform()->GetWorldTM());
                    m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
                }
            });
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void PointcloudComponent::Deactivate()
    {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        if (m_featureProcessor)
        {
            m_featureProcessor->ReleasePointcloud(m_pointcloudHandle);
        }
    }

    void PointcloudComponent::OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world)
    {
        AZ_UNUSED(local);
        if (m_pointcloudHandle != PointcloudFeatureProcessorInterface::InvalidPointcloudHandle)
        {
            m_featureProcessor->SetTransform(m_pointcloudHandle, world);
        }
    }
} // namespace Pointcloud
