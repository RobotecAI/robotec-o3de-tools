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
#include <3rd/happly.h>
namespace Pointcloud
{

    void PointcloudComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudComponent, AZ::Component>()
                ->Version(0)
                ->Field("PointcloudAsset", &PointcloudComponent::m_pointcloudAsset)
                ->Field("PointSize", &PointcloudComponent::m_pointSize);
            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PointcloudComponent>("PointcloudComponent", "PointcloudComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "PointcloudComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudComponent::m_pointcloudAsset,
                        "Pointcloud Asset",
                        "Asset containing the pointcloud data")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudComponent::m_pointSize,
                        "Point Size",
                        "Size of the points in the pointcloud");
            }
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
                    m_pointcloudAsset.QueueLoad();
                    m_pointcloudAsset.BlockUntilLoadComplete();

                    AZStd::vector<AZStd::vector<PointcloudAsset::CloudVertex>> cloudVertexDataChunks;

                    m_pointcloudHandle = m_featureProcessor->AcquirePointcloud(m_pointcloudAsset->m_data);

                    if (m_pointcloudHandle != PointcloudFeatureProcessorInterface::InvalidPointcloudHandle)
                    {
                        m_featureProcessor->SetTransform(m_pointcloudHandle, m_entity->GetTransform()->GetWorldTM());
                        m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
                    }

                }
            });
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void PointcloudComponent::Deactivate()
    {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        m_featureProcessor->ReleasePointcloud(m_pointcloudHandle);
    }

    void PointcloudComponent::OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world)
    {
        AZ_UNUSED(local);
        if (m_pointcloudHandle != PointcloudFeatureProcessorInterface::InvalidPointcloudHandle)
        {
            m_featureProcessor->SetTransform(m_pointcloudHandle, m_entity->GetTransform()->GetWorldTM());
        }
    }

} // namespace Pointcloud
