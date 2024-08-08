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
#include <Clients/GrassComponent.h>

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
                    ->Field("Total Verticies", &GrassEditorComponent::m_totalVertices)
                    ->Field("Shader Parameters", &GrassEditorComponent::m_shaderParameters);
            AZ::EditContext *editContext = serializeContext->GetEditContext();
            if (editContext) {
                editContext->Class<GrassEditorComponent>("GrassEditorComponent", "GrassEditorComponent")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "GrassEditorComponent")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GrassEditorComponent::m_totalVertices,
                                        "Total Vertices", "Total number of vertices in shader")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &GrassEditorComponent::m_shaderParameters,
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
        gameEntity->CreateComponent<GrassComponent>(m_totalVertices, m_shaderParameters);
    }

    void GrassEditorComponent::LoadParameters(const AZStd::vector<ShaderParameterUnion> &shaderParameters) {
        // for for look for matching parameters names and types if they exist continue else add them
        for (const auto &param : shaderParameters) {
            bool found = false;
            for (auto &existingParam : this->m_shaderParameters) {
                if (existingParam.m_parameterName == param.m_parameterName.GetStringView()) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                this->m_shaderParameters.push_back(ShaderParameter(param));
            }
        }

    }

    AZStd::vector<ShaderParameterUnion> GrassEditorComponent::GetConstants() const {
        AZStd::vector<ShaderParameterUnion> shaderParameterUnions;
        for (const auto &param : m_shaderParameters) {
            shaderParameterUnions.push_back(param.ToShaderParameterUnion());
        }
        return shaderParameterUnions;
    }

    void GrassEditorComponent::PushNewParameters() {
        m_featureProcessor->SetParameters(GetConstants());
    }
    
    AZ::Crc32 GrassEditorComponent::ForceUpdate() {
        LoadParameters(m_featureProcessor->GetParameters());

        if (m_featureProcessor) {
            AZ_Printf("GrassEditorComponent", "Setting total vertices to: %d", m_totalVertices);
            m_featureProcessor->ForceUpdate(m_totalVertices);
            PushNewParameters();
        }

        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    // void GrassEditorComponent::OnTransformChanged(const AZ::Transform &local, const AZ::Transform &world) {
    //     if (m_featureProcessor) {
    //         m_featureProcessor->SetTransform(world);
    //     }
    // }
}
