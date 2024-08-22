#pragma once

#include "PointcloudComponentController.h"

#include "../../Clients/PointcloudComponent.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Scene/Scene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <Pointcloud/PointcloudComponentControllerConfigurationBus.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{

    using PointcloudEditorComponentBase =
        AzToolsFramework::Components::EditorComponentAdapter<PointcloudComponentController, PointcloudComponent, PointcloudComponentConfig>;

    class PointcloudEditorComponent
        : public PointcloudEditorComponentBase
        , private AZ::TransformNotificationBus::Handler
        , private AzToolsFramework::EditorEntityInfoNotificationBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(
            PointcloudEditorComponent, "{5950AC6B-75F3-4E0F-BA5C-17C877013722}", AzToolsFramework::Components::EditorComponentBase);
        PointcloudEditorComponent() = default;
        explicit PointcloudEditorComponent(const PointcloudComponentConfig& configuration);
        ~PointcloudEditorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;
        bool ShouldActivateController() const override;

    private:
        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // AzToolsFramework::EditorEntityInfoNotificationBus overrides ...
        void OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible) override;
    };
} // namespace Pointcloud
