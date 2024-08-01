#pragma once

#include <Atom/RPI.Reflect/ResourcePoolAssetCreator.h>
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Scene/Scene.h>
#include <Pointcloud/PointcloudAsset.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Pointcloud/PointcloudTypeIds.h>
namespace Pointcloud
{

    class PointcloudComponent
        : public AZ::Component
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(PointcloudComponent, PointcloudComponentTypeId);
        PointcloudComponent() = default;
        PointcloudComponent(const AZ::Data::Asset<PointcloudAsset>& pointcloudAsset, const float pointSize);
        ~PointcloudComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // Component interface overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        AZ::Data::Asset<PointcloudAsset> m_pointcloudAsset;
        float m_pointSize = 1.0f;
        PointcloudFeatureProcessorInterface* m_featureProcessor = nullptr;
        AZ::RPI::Scene* m_scene = nullptr;
        PointcloudFeatureProcessorInterface::PointcloudHandle m_pointcloudHandle =
            PointcloudFeatureProcessorInterface::InvalidPointcloudHandle;
    };
} // namespace Pointcloud
