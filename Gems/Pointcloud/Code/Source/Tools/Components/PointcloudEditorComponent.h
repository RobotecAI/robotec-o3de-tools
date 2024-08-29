#pragma once

#include "PointcloudComponentController.h"

#include "../../Clients/PointcloudComponent.h"
#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>

#include <AzFramework/Visibility/BoundsBus.h>

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
        , public AzFramework::BoundsRequestBus::Handler
        , public AzToolsFramework::EditorComponentSelectionRequestsBus::Handler
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

        AZ::Aabb GetWorldBounds() const override;
        AZ::Aabb GetLocalBounds() const override;

        AZ::Aabb GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo) override;
        bool EditorSelectionIntersectRayViewport(
            const AzFramework::ViewportInfo& viewportInfo, const AZ::Vector3& src, const AZ::Vector3& dir, float& distance) override;

        bool SupportsEditorRayIntersect() override;
        bool SupportsEditorRayIntersectViewport(const AzFramework::ViewportInfo& viewportInfo) override;

    private:
        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // AzToolsFramework::EditorEntityInfoNotificationBus overrides ...
        void OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible) override;
    };
} // namespace Pointcloud
