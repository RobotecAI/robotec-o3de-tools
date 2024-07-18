#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Scene/Scene.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Pointcloud/PointcloudAsset.h>
#include <Atom/RPI.Reflect/ResourcePoolAssetCreator.h>
namespace Pointcloud
{

    class PointcloudComponent
        : public AZ::Component
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(PointcloudComponent, "{0190c091-83aa-7c6e-a6da-5efea1f23473}");
        PointcloudComponent() = default;
        ~PointcloudComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // Component interface overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        float m_pointSize = 1.0f;
        AZ::Data::Asset<PointcloudAsset> m_pointcloudAsset;
        PointcloudFeatureProcessorInterface *m_featureProcessor = nullptr;
        AZ::RPI::Scene *m_scene = nullptr;
        PointcloudFeatureProcessorInterface::PointcloudHandle m_pointcloudHandle = PointcloudFeatureProcessorInterface::InvalidPointcloudHandle;

    };
} // namespace Pointcloud
