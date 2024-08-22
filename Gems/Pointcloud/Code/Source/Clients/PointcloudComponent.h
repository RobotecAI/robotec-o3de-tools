#pragma once

#include "AzFramework/Components/ComponentAdapter.h"
#include "Tools/Components/PointcloudComponentController.h"
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
    using PointcloudComponentBase = AzFramework::Components::ComponentAdapter<PointcloudComponentController, PointcloudComponentConfig>;

    class PointcloudComponent
        : public PointcloudComponentBase
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(PointcloudComponent, PointcloudComponentTypeId);
        PointcloudComponent() = default;
        PointcloudComponent(const PointcloudComponentConfig& config);
        ~PointcloudComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // Component interface overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;
    };
} // namespace Pointcloud
