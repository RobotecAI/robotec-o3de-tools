#include "PointcloudEditorComponent.h"
#include "Clients/PointcloudComponent.h"
#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Entity/EntityContext.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <AzToolsFramework/ToolsComponents/EditorVisibilityBus.h>
#include <Render/PointcloudFeatureProcessor.h>

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
                editContext->Class<PointcloudEditorComponent>("PointcloudEditorComponent", "PointcloudEditorComponent")
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
    }

    void PointcloudEditorComponent::Deactivate()
    {
        PointcloudEditorComponentBase::Deactivate();
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusDisconnect();
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        AzFramework::BoundsRequestBus::Handler::BusDisconnect();
    }
    bool PointcloudEditorComponent::ShouldActivateController() const
    {
        return false;
    }

    AZ::Aabb PointcloudEditorComponent::GetWorldBounds() const
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        AZ::Aabb bounds = m_controller.GetBounds();
        bounds.ApplyTransform(transform);
        return bounds;
    }

    AZ::Aabb PointcloudEditorComponent::GetLocalBounds() const
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        AZ::Aabb bounds = m_controller.GetBounds();
        bounds.ApplyTransform(transform);
        return bounds;
    }

    AZ::Aabb PointcloudEditorComponent::GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo)
    {
        return GetWorldBounds();
    }

    // Based on https://github.com/erich666/GraphicsGems/blob/master/gems/RayBox.c
    bool PointcloudEditorComponent::EditorSelectionIntersectRayViewport(
        const AzFramework::ViewportInfo& viewportInfo, const AZ::Vector3& src, const AZ::Vector3& dir, float& distance)
    {
        bool inside = true;
        bool quadrant[3];
        int whichPlane;
        AZ::Vector3 minB = m_controller.GetBounds().GetMin();
        AZ::Vector3 maxB = m_controller.GetBounds().GetMax();
        AZ::Vector3 origin = src;
        AZ::Vector3 candidatePlane;
        AZ::Vector3 maxT;
        AZ::Vector3 coord;

        for (int i = 0; i < 3; i++)
        {
            if (origin.GetElement(i) < minB.GetElement(i))
            {
                quadrant[i] = false;
                candidatePlane.SetElement(i, minB.GetElement(i));
                inside = false;
            }
            else if (origin.GetElement(i) > maxB.GetElement(i))
            {
                quadrant[i] = true;
                candidatePlane.SetElement(i, maxB.GetElement(i));
                inside = false;
            }
            else
            {
                quadrant[i] = true;
            }
        }

        if (inside)
        {
            coord = origin;
            // When inside the bounding box, compute distance from ray origin to a point on the box
            distance = (origin - (minB + (maxB - minB) * 0.5f)).GetLength();
            return true;
        }

        for (int i = 0; i < 3; i++)
        {
            if (!quadrant[i] && dir.GetElement(i) != 0)
            {
                maxT.SetElement(i, (candidatePlane.GetElement(i) - origin.GetElement(i)) / dir.GetElement(i));
            }
            else
            {
                maxT.SetElement(i, -1);
            }
        }

        whichPlane = 0;
        for (int i = 1; i < 3; i++)
        {
            if (maxT.GetElement(whichPlane) < maxT.GetElement(i))
            {
                whichPlane = i;
            }
        }

        if (maxT.GetElement(whichPlane) < 0)
        {
            return false;
        }

        for (int i = 0; i < 3; i++)
        {
            if (whichPlane != i)
            {
                coord.SetElement(i, origin.GetElement(i) + maxT.GetElement(whichPlane) * dir.GetElement(i));
                if (coord.GetElement(i) < minB.GetElement(i) || coord.GetElement(i) > maxB.GetElement(i))
                {
                    return false;
                }
            }
            else
            {
                coord.SetElement(i, candidatePlane.GetElement(i));
            }
        }

        // Compute distance from ray origin to intersection point
        distance = (coord - origin).GetLength();

        return true;
    }

    bool PointcloudEditorComponent::SupportsEditorRayIntersect()
    {
        return true;
    }
    bool PointcloudEditorComponent::SupportsEditorRayIntersectViewport(const AzFramework::ViewportInfo& viewportInfo)
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

} // namespace Pointcloud
