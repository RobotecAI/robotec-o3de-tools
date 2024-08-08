#pragma once

#include <Atom/RPI.Reflect/ResourcePoolAssetCreator.h>
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Scene/Scene.h>
#include "../Tools/Components/ShaderParameter.h"
#include <Grass/GrassFeatureProcessorInterface.h>
#include <Grass/GrassTypeIds.h>
namespace Grass
{

    class GrassComponent
        : public AZ::Component
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(GrassComponent, GrassComponentTypeId);
        GrassComponent() = default;
        GrassComponent(uint32_t m_totalVertices,AZStd::vector<ShaderParameter> shaderParameters);
        ~GrassComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // Component interface overrides ...
        void Activate() override;
        void Deactivate() override;

        AZStd::vector<ShaderParameterUnion> GetConstants() const;

    private:
        void PushNewParameters();

        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;
        uint32_t m_totalVertices;
        AZStd::vector<ShaderParameter> m_shaderParameters;
        GrassFeatureProcessorInterface* m_featureProcessor = nullptr;
        AZ::RPI::Scene* m_scene = nullptr;
    };
} // namespace Grass
