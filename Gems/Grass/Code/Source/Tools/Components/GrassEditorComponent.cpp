#include "GrassEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <QMessageBox>
#include <QFileDialog>
#include <random>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityContext.h>
#include <AzFramework/Scene/Scene.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <AzFramework/Scene/SceneSystemInterface.h>

#include <AzFramework/Scene/SceneSystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Render/GrassFeatureProcessor.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/UI/UICore/WidgetHelpers.h>

namespace Grass {

    void GrassEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType &required) {
        required.push_back(AZ_CRC_CE("TransformService"));
    }


    inline void registerParameterTypeEnum(AZ::SerializeContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            context->Enum<ParameterType>()
                ->Value("Float", ParameterType::Float)
                ->Value("Float2", ParameterType::Float2)
                ->Value("Float3", ParameterType::Float3)
                ->Value("uint", ParameterType::uint);
        }
    }

    void GrassEditorComponent::Reflect(AZ::ReflectContext *context) {

        AZ::SerializeContext *serializeContext = azrtti_cast<AZ::SerializeContext *>(context);
        registerParameterTypeEnum(serializeContext);
        ShaderParameter::Reflect(context);
        if (serializeContext) {
            serializeContext->Class<GrassEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                    ->Version(2)
                    ->Field("Point Size", &GrassEditorComponent::m_pointSize)
                    ->Field("Move To Centroid", &GrassEditorComponent::m_moveToCentroid)
                    ->Field("Total Verticies", &GrassEditorComponent::m_totalVertices)
                    ->Field("Shader Parameters", &GrassEditorComponent::shaderParameters);
            AZ::EditContext *editContext = serializeContext->GetEditContext();
            if (editContext) {
                editContext->Class<GrassEditorComponent>("GrassEditorComponent", "GrassEditorComponent")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "GrassEditorComponent")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GrassEditorComponent::m_totalVertices,
                                        "Total Vertices", "Total number of vertices in shader")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GrassEditorComponent::shaderParameters,
                                      "Shader Parameters", "Shader parameters to set")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GrassEditorComponent::PushNewParameters)
                        // ->DataElement(AZ::Edit::UIHandlers::Default, &GrassEditorComponent::m_pointSize,
                        //               "Point Size", "Size of the points in the grass")
                        // ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GrassEditorComponent::OnSetPointSize)
                        ->UIElement(AZ::Edit::UIHandlers::Button, "ForceUpdate", "")
                        ->Attribute(AZ::Edit::Attributes::ButtonText, "ForceUpdate")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GrassEditorComponent::ForceUpdate);
            }
        }
    }

    void GrassEditorComponent::Activate() {
        m_scene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
        if (m_scene) {
            m_featureProcessor = m_scene->EnableFeatureProcessor<GrassFeatureProcessor>();

            AZ_Assert(m_featureProcessor, "Failed to enable GrassFeatureProcessorInterface.");

        }

        // AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void GrassEditorComponent::Deactivate() {
        // AZ::TransformNotificationBus::Handler::BusDisconnect();
        if (m_scene) {
            m_scene->DisableFeatureProcessor<GrassFeatureProcessor>();
        }
    }



    void GrassEditorComponent::BuildGameEntity([[maybe_unused]] AZ::Entity *gameEntity) {
    }

    void GrassEditorComponent::LoadParameters(const AZStd::vector<ShaderParameterUnion> &shaderParameters) {
        // for for look for matching parameters names and types if they exist continue else add them
        for (const auto &param : shaderParameters) {
            bool found = false;
            for (auto &existingParam : this->shaderParameters) {
                if (existingParam.m_parameterName == param.m_parameterName.GetStringView()) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                this->shaderParameters.push_back(ShaderParameter(param));
            }
        }

    }

    AZStd::vector<ShaderParameterUnion> GrassEditorComponent::GetConstants() const {
        AZStd::vector<ShaderParameterUnion> shaderParameterUnions;
        for (const auto &param : shaderParameters) {
            shaderParameterUnions.push_back(param.ToShaderParameterUnion());
        }
        return shaderParameterUnions;
    }

    void GrassEditorComponent::PushNewParameters() {
        m_featureProcessor->SetParameters(GetConstants());
    }
    
    AZ::Crc32 GrassEditorComponent::ForceUpdate() {
        LoadParameters(m_featureProcessor->GetParameters());
        // //auto vertices = plyIn.getVertexPositions();
        // std::vector<std::array<double, 3>> vertices;
        // // fill with grid of points
        // uint32_t width = 2;
        // uint32_t height = width;
        // double cellSize = 0.3;
        //
        // std::normal_distribution<double> distribution(0.0, 0.3);
        // auto m_random = std::mt19937(std::random_device{}());
        // for (uint32_t i = 0; i < width; i++) {
        //     for (uint32_t j = 0; j < height; j++) {
        //         AZ::Vector3 noise(distribution(m_random), distribution(m_random), 0);
        //         vertices.push_back({i * cellSize + noise.GetX(), j * cellSize + noise.GetY(), 0});
        //     }
        // }
        // vertices.push_back({-3,-3,0});
        //
        // // random shuffee vertices
        // std::shuffle(vertices.begin(), vertices.end(), m_random);
        //
        // printf("Loaded %lu vertices\n", vertices.size());
        //
        // std::vector<std::array<unsigned char, 3>> colors(vertices.size(), {255, 255, 255});
        //
        //
        // AZStd::vector<GrassFeatureProcessor::CloudVertex> cloudVertexData;
        // for (int i = 0; i < vertices.size(); i++) {
        //     GrassFeatureProcessor::CloudVertex vertex;
        //     vertex.m_position = {static_cast<float>(vertices[i][0]),
        //                          static_cast<float>(vertices[i][1]),
        //                          static_cast<float>(vertices[i][2])};
        //     if (i < colors.size()) {
        //         unsigned char r = colors[i][0];
        //         unsigned char g = colors[i][1];
        //         unsigned char b = colors[i][2];
        //         AZ::Color m_color {r, g, b, 255};
        //         vertex.m_color = m_color.ToU32();
        //
        //     }
        //     cloudVertexData.push_back(vertex);
        // }
        if (m_featureProcessor) {
            AZ_Printf("GrassEditorComponent", "Setting total vertices to: %d", m_totalVertices);
            m_featureProcessor->ForceUpdate(m_totalVertices);
            PushNewParameters();
            // m_featureProcessor->SetTransform(GetWorldTM());
            // m_featureProcessor->SetPointSize(m_pointSize);
        }

        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    // void GrassEditorComponent::OnTransformChanged(const AZ::Transform &local, const AZ::Transform &world) {
    //     if (m_featureProcessor) {
    //         m_featureProcessor->SetTransform(world);
    //     }
    // }
}
