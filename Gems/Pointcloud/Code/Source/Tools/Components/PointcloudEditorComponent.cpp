#include "PointcloudEditorComponent.h"
#include "Clients/PointcloudComponent.h"

#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <Render/PointcloudFeatureProcessor.h>
#include <Viewport/ViewportMessages.h>
#include <ViewportSelection/EditorSelectionUtil.h>

namespace Pointcloud
{

    void PointcloudEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void PointcloudEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        PointcloudEditorComponentBase::Reflect(context);
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<PointcloudEditorComponent, PointcloudEditorComponentBase>()->Version(3);
            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<PointcloudEditorComponent>("Pointcloud", "Visualize a point cloud data")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "PointcloudEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void PointcloudEditorComponent::Activate()
    {
        PointcloudEditorComponentBase::Activate();
        PointcloudComponentConfig config = m_controller.GetConfiguration();
        config.m_editorEntityId = GetEntityId();
        AZ_Assert(config.m_editorEntityId.IsValid(), "Pointcloud component got an invalid entity id");
        m_controller.SetConfiguration(config);
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusConnect();
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        bool visible = true;
        AzToolsFramework::EditorEntityInfoRequestBus::EventResult(
            visible, GetEntityId(), &AzToolsFramework::EditorEntityInfoRequestBus::Events::IsVisible);
        m_controller.SetVisibility(visible);
        AzFramework::BoundsRequestBus::Handler::BusConnect(GetEntityId());
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void PointcloudEditorComponent::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        PointcloudEditorComponentBase::Deactivate();
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusDisconnect();
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        AzFramework::BoundsRequestBus::Handler::BusDisconnect();
    }

    AZ::Aabb PointcloudEditorComponent::GetWorldBounds()
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        AZ::Aabb bounds = m_controller.GetBounds();
        if (bounds.IsValid())
        {
            bounds.ApplyTransform(transform);
        }
        return bounds;
    }

    AZ::Aabb PointcloudEditorComponent::GetLocalBounds()
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        AZ::Aabb bounds = m_controller.GetBounds();
        if (bounds.IsValid())
        {
            bounds.ApplyTransform(transform);
        }
        return bounds;
    }

    AZ::Aabb PointcloudEditorComponent::GetEditorSelectionBoundsViewport([[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        return GetWorldBounds();
    }

    bool PointcloudEditorComponent::EditorSelectionIntersectRayViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, const AZ::Vector3& src, const AZ::Vector3& dir, float& distance)
    {
        return AzToolsFramework::AabbIntersectRay(src, dir, GetWorldBounds(), distance);
    }

    bool PointcloudEditorComponent::SupportsEditorRayIntersect()
    {
        return true;
    }

    bool PointcloudEditorComponent::SupportsEditorRayIntersectViewport([[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        return true;
    }

    void PointcloudEditorComponent::OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible)
    {
        m_controller.OnEntityInfoUpdatedVisibility(entityId, visible);
    }

    void PointcloudEditorComponent::OnTransformChanged([[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        m_controller.OnTransformChanged(local, world);
    }

    void PointcloudEditorComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        if (!IsSelected())
        {
            return;
        }

        const AZ::Aabb bounds = m_controller.GetBounds();
        if (!bounds.IsValid())
        {
            return;
        }

        AZ::Transform worldTM = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTM, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        debugDisplay.PushMatrix(worldTM);
        debugDisplay.SetColor(AZ::Colors::White);
        debugDisplay.DrawWireBox(bounds.GetMin(), bounds.GetMax());
        debugDisplay.PopMatrix();
    }

} // namespace Pointcloud
