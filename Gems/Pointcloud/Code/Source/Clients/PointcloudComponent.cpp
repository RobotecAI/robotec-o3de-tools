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

                //                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::OnSetPointSize)
                //                    ->UIElement(AZ::Edit::UIHandlers::Button, "LoadCloud", "")
                //                    ->Attribute(AZ::Edit::Attributes::ButtonText, "LoadCloud")
                //                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::LoadCloud);
            }
        }
    }

    void PointcloudComponent::Activate()
    {
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                m_scene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
                if (m_scene)
                {
                    m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();
                    AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
                    m_pointcloudAsset.QueueLoad();
                    m_pointcloudAsset.BlockUntilLoadComplete();
                    m_featureProcessor->SetCloud(m_pointcloudAsset->m_data);
                    m_featureProcessor->SetTransform(AZ::Transform::Identity());
                    m_featureProcessor->SetPointSize(m_pointSize);

//                    happly::PLYData plyIn("/home/michalpelka/github/KubotaAgricultureSimulator/project/Assets/cloud.ply");
//                    auto vertices = plyIn.getVertexPositions();
//
//                    std::vector<std::array<unsigned char, 3>> colors;
//                    try {
//                        colors = plyIn.getVertexColors();
//                    } catch (std::exception &e) {
//                        AZ_Printf("PointcloudEditorComponent", "No colors in the file");
//                    }
//
//                    if (true) {
//                        double centroid[3] = {0, 0, 0};
//                        for (int i = 0; i < vertices.size(); i++) {
//                            centroid[0] += vertices[i][0];
//                            centroid[1] += vertices[i][1];
//                            centroid[2] += vertices[i][2];
//                        }
//                        centroid[0] /= vertices.size();
//                        centroid[1] /= vertices.size();
//                        centroid[2] /= vertices.size();
//                        for (int i = 0; i < vertices.size(); i++) {
//                            vertices[i][0] -= centroid[0];
//                            vertices[i][1] -= centroid[1];
//                            vertices[i][2] -= centroid[2];
//                        }
//
//                    }
//
//                    AZStd::vector<PointcloudAsset::CloudVertex> cloudVertexData;
//                    for (int i = 0; i < vertices.size(); i++) {
//                        PointcloudAsset::CloudVertex vertex;
//                        vertex.m_position = {static_cast<float>(vertices[i][0]),
//                                              static_cast<float>(vertices[i][1]),
//                                              static_cast<float>(vertices[i][2])};
//                        if (i < colors.size()) {
//                            unsigned char r = colors[i][0];
//                            unsigned char g = colors[i][1];
//                            unsigned char b = colors[i][2];
//                            AZ::Color m_color {r, g, b, 255};
//                            vertex.m_color = m_color.ToU32();
//
//                        }
//                        cloudVertexData.push_back(vertex);
//                    }
//                    if (m_featureProcessor) {
//                        AZ_Printf("PointcloudEditorComponent", "Setting cloud, size %d", cloudVertexData.size());
//                        m_featureProcessor->SetCloud(cloudVertexData);
//                        m_featureProcessor->SetTransform(AZ::Transform::Identity());
//                        m_featureProcessor->SetPointSize(m_pointSize);
//                    }
//


//                    m_featureProcessor->AquirePointcloud(m_pointcloudAsset);
                }
            });
    }

    void PointcloudComponent::Deactivate()
    {
    }

    void PointcloudComponent::OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world)
    {
        AZ_UNUSED(local);
        AZ_UNUSED(world);
    }

} // namespace Pointcloud
