#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Scene/Scene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
namespace Pointcloud
{

    class PointcloudComponent
        : public AZ::Component
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(PointcloudComponent, "{018fba15-560f-78cb-afb4-cf4d00cefc17}");
        PointcloudComponent() = default;
        ~PointcloudComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // Component interface overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

//        AZ::Crc32 OnSetPointSize();
//        AZ::Crc32 LoadCloud();
//        float m_pointSize = 1.0f;
//        bool m_moveToCentroid{ true };
//        PointcloudFeatureProcessorInterface* m_featureProcessor = nullptr;
//        AZ::RPI::Scene* m_scene = nullptr;
//        AZStd::vector<PointcloudFeatureProcessorInterface::CloudVertex> m_cloudData;
    };
} // namespace Pointcloud
