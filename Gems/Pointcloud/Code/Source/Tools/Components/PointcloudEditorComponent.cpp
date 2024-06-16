#include "PointcloudEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <QMessageBox>
#include <QFileDialog>
#include <3rd/happly.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityContext.h>
#include <AzFramework/Scene/Scene.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <AzFramework/Scene/SceneSystemInterface.h>

#include <AzFramework/Scene/SceneSystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Render/PointcloudFeatureProcessor.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/UI/UICore/WidgetHelpers.h>

namespace Pointcloud {

    void PointcloudEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType &required) {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void PointcloudEditorComponent::Reflect(AZ::ReflectContext *context) {
        AZ::SerializeContext *serializeContext = azrtti_cast<AZ::SerializeContext *>(context);
        if (serializeContext) {
            serializeContext->Class<PointcloudEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                    ->Version(2)
                    ->Field("Point Size", &PointcloudEditorComponent::m_pointSize)
                    ->Field("Move To Centroid", &PointcloudEditorComponent::m_moveToCentroid);
            AZ::EditContext *editContext = serializeContext->GetEditContext();
            if (editContext) {
                editContext->Class<PointcloudEditorComponent>("PointcloudEditorComponent", "PointcloudEditorComponent")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "PointcloudEditorComponent")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &PointcloudEditorComponent::m_moveToCentroid,
                                      "Move To Centroid", "Move the entity to the centroid of the pointcloud during load")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &PointcloudEditorComponent::m_pointSize,
                                      "Point Size", "Size of the points in the pointcloud")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::OnSetPointSize)
                        ->UIElement(AZ::Edit::UIHandlers::Button, "LoadCloud", "")
                        ->Attribute(AZ::Edit::Attributes::ButtonText, "LoadCloud")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::LoadCloud);
            }
        }
    }

    void PointcloudEditorComponent::Activate() {
        m_scene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
        if (m_scene) {
            m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();

            AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");

        }

        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void PointcloudEditorComponent::Deactivate() {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        if (m_scene) {
            m_scene->DisableFeatureProcessor<PointcloudFeatureProcessor>();
        }
    }

    void PointcloudEditorComponent::BuildGameEntity([[maybe_unused]] AZ::Entity *gameEntity) {
    }

    AZ::Crc32 PointcloudEditorComponent::OnSetPointSize() {
        if (m_featureProcessor) {
            m_featureProcessor->SetPointSize(m_pointSize);
        }
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    AZ::Crc32 PointcloudEditorComponent::LoadCloud() {
        //auto vertices = plyIn.getVertexPositions();
        std::vector<std::array<double, 3>> vertices;
        // fill with grid of points
        for (double i = 0; i < 10; i++) {
            for (double j = 0; j < 1; j++) {
                vertices.push_back({i, j, 0});
            }
        }

        std::vector<std::array<unsigned char, 3>> colors(vertices.size(), {255, 255, 255});

//        try {
//            colors = plyIn.getVertexColors();
//        } catch (std::exception &e) {
//            AZ_Printf("PointcloudEditorComponent", "No colors in the file");
//        }

        if (m_moveToCentroid) {
            double centroid[3] = {0, 0, 0};
            for (int i = 0; i < vertices.size(); i++) {
                centroid[0] += vertices[i][0];
                centroid[1] += vertices[i][1];
                centroid[2] += vertices[i][2];
            }
            centroid[0] /= vertices.size();
            centroid[1] /= vertices.size();
            centroid[2] /= vertices.size();
            for (int i = 0; i < vertices.size(); i++) {
                vertices[i][0] -= centroid[0];
                vertices[i][1] -= centroid[1];
                vertices[i][2] -= centroid[2];
            }

        }

        AZStd::vector<PointcloudFeatureProcessor::CloudVertex> cloudVertexData;
        for (int i = 0; i < vertices.size(); i++) {
            PointcloudFeatureProcessor::CloudVertex vertex;
            vertex.m_position = {static_cast<float>(vertices[i][0]),
                                 static_cast<float>(vertices[i][1]),
                                 static_cast<float>(vertices[i][2])};
            if (i < colors.size()) {
                unsigned char r = colors[i][0];
                unsigned char g = colors[i][1];
                unsigned char b = colors[i][2];
                AZ::Color m_color {r, g, b, 255};
                vertex.m_color = m_color.ToU32();

            }
            cloudVertexData.push_back(vertex);
        }
        if (m_featureProcessor) {
            AZ_Printf("PointcloudEditorComponent", "Setting cloud, size %d", cloudVertexData.size());
            m_featureProcessor->SetCloud(cloudVertexData);
            m_featureProcessor->SetTransform(GetWorldTM());
            m_featureProcessor->SetPointSize(m_pointSize);
        }

        return AZ::Edit::PropertyRefreshLevels::None;
    }

    void PointcloudEditorComponent::OnTransformChanged(const AZ::Transform &local, const AZ::Transform &world) {
        if (m_featureProcessor) {
            m_featureProcessor->SetTransform(world);
        }
    }
}
