#include "PointcloudEditorComponent.h"
#include "Clients/PointcloudComponent.h"
#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Entity/EntityContext.h>
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
    }

    void PointcloudEditorComponent::Deactivate()
    {
        PointcloudEditorComponentBase::Deactivate();
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusDisconnect();
    }
    bool PointcloudEditorComponent::ShouldActivateController() const
    {
        return false;
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
