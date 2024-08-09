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
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<PointcloudEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("Point Size", &PointcloudEditorComponent::m_pointSize)
                ->Field("PointcloudAsset", &PointcloudEditorComponent::m_pointcloudAsset)
                ->Field("NumPoints", &PointcloudEditorComponent::m_numPoints);
            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<PointcloudEditorComponent>("PointcloudEditorComponent", "PointcloudEditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "PointcloudEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudEditorComponent::m_pointSize,
                        "Point Size",
                        "Size of the points in the pointcloud")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::OnSetPointSize)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudEditorComponent::m_pointcloudAsset,
                        "Pointcloud Asset",
                        "Asset containing the pointcloud data")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::OnAssetChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudEditorComponent::m_numPoints,
                        "NumPoints",
                        "Number of points in the pointcloud")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, true);
            }
        }
    }

    void PointcloudEditorComponent::Activate()
    {
        m_scene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
        if (m_scene)
        {
            m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();

            AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
        }
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                m_scene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
                if (m_scene && m_pointcloudAsset)
                {
                    m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();
                    AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
                    if (m_pointcloudAsset)
                    {
                        m_pointcloudHandle = m_featureProcessor->AcquirePointcloudFromAsset(m_pointcloudAsset);
                        m_featureProcessor->SetTransform(m_pointcloudHandle, m_entity->GetTransform()->GetWorldTM());
                        m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
                    }
                }
            });
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusConnect();
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        PointcloudEditorComponentConfigurationBus::Handler::BusConnect(GetEntityId());
    }

    void PointcloudEditorComponent::Deactivate()
    {
        PointcloudEditorComponentConfigurationBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusDisconnect();
        if (m_featureProcessor)
        {
            m_featureProcessor->ReleasePointcloud(m_pointcloudHandle);
        }
    }

    void PointcloudEditorComponent::BuildGameEntity([[maybe_unused]] AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<PointcloudComponent>(m_pointcloudAsset, m_pointSize);
    }

    AZ::Crc32 PointcloudEditorComponent::OnSetPointSize()
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
        }
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    AZ::Crc32 PointcloudEditorComponent::OnAssetChanged()
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->ReleasePointcloud(m_pointcloudHandle);
            if (m_pointcloudAsset.GetId().IsValid())
            {
                m_pointcloudHandle = m_featureProcessor->AcquirePointcloudFromAsset(m_pointcloudAsset);
                m_featureProcessor->SetTransform(m_pointcloudHandle, m_entity->GetTransform()->GetWorldTM());
                m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
            }
            else
            {
                m_numPoints = 0;
            }
        }
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    void PointcloudEditorComponent::OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible)
    {
        if (entityId == GetEntityId())
        {
            m_featureProcessor->SetVisibility(m_pointcloudHandle, visible);
        }
    }

    void PointcloudEditorComponent::OnTransformChanged([[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->SetTransform(m_pointcloudHandle, world);
        }
    }

    void PointcloudEditorComponent::SetPointcloudAsset(AZ::Data::Asset<PointcloudAsset> asset)
    {
        m_pointcloudAsset = asset;
    }
    void PointcloudEditorComponent::SetPointSize(float pointSize)
    {
        m_pointSize = pointSize;
    }

} // namespace Pointcloud
