#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <AzCore/Component/TransformBus.h>
#include <Grass/GrassFeatureProcessorInterface.h>
#include <AzFramework/Scene/Scene.h>
#include "ShaderParameter.h"
namespace Grass
{

    class GrassEditorComponent : public AzToolsFramework::Components::EditorComponentBase
        // private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(GrassEditorComponent, "{223c9238-9fda-4629-a726-824a1cd75ee7}");
        GrassEditorComponent() = default;
        ~GrassEditorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;
        void LoadParameters(const AZStd::vector<ShaderParameterUnion>& shaderParameters);
        AZStd::vector<ShaderParameterUnion> GetConstants() const;
        void PushNewParameters();

        uint32_t m_totalVertices = 3;

    private:

        //AZ::TransformNotificationBus::Handler overrides ...
        // void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;
        AZStd::vector<ShaderParameter> shaderParameters;
        // AZ::Crc32 OnSetPointSize();
        AZ::Crc32 ForceUpdate();

        float m_pointSize = 1.0f;
        bool m_moveToCentroid {true};
        GrassFeatureProcessorInterface *m_featureProcessor = nullptr;
        AZ::RPI::Scene *m_scene = nullptr;
    };
} // namespace Grass
