#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Scene/Scene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <Pointcloud/PointcloudComponentConfigurationBus.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{

    class PointcloudEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AZ::TransformNotificationBus::Handler
        , private AzToolsFramework::EditorEntityInfoNotificationBus::Handler
        , private PointcloudEditorComponentConfigurationBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(PointcloudEditorComponent, PointcloudEditorComponentTypeId);
        PointcloudEditorComponent() = default;
        ~PointcloudEditorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // AzToolsFramework::EditorEntityInfoNotificationBus overrides ...
        void OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible) override;

        // PointcloudEditorComponentConfigurationBus::Handler overrides ...
        void SetPointcloudAsset(AZ::Data::Asset<PointcloudAsset> asset) override;
        void SetPointSize(float pointSize) override;

        AZ::Crc32 OnSetPointSize();
        AZ::Crc32 OnAssetChanged();

        float m_pointSize = 1.0f;
        uint32_t m_numPoints = 0;
        PointcloudFeatureProcessorInterface* m_featureProcessor = nullptr;
        AZ::RPI::Scene* m_scene = nullptr;
        PointcloudFeatureProcessorInterface::PointcloudHandle m_pointcloudHandle =
            PointcloudFeatureProcessorInterface::InvalidPointcloudHandle;
        AZ::Data::Asset<PointcloudAsset> m_pointcloudAsset;
    };
} // namespace Pointcloud
