#include "PointcloudComponentController.h"
#include "Clients/PointcloudComponent.h"
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Entity/EntityContext.h>
#include <Render/PointcloudFeatureProcessor.h>

namespace Pointcloud
{

    PointcloudComponentController::PointcloudComponentController(const PointcloudComponentConfig& config)
    {
        m_config = config;
    }

    void PointcloudComponentController::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void PointcloudComponentController::Init()
    {
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                m_scene = AZ::RPI::Scene::GetSceneForEntityId(m_config.m_editorEntityId);
                if (m_scene)
                {
                    m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();

                    AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
                    OnAssetChanged();
                }
            });
    }

    void PointcloudComponentController::SetConfiguration(const PointcloudComponentConfig& config)
    {
        m_config = config;
        OnAssetChanged();
    }

    const PointcloudComponentConfig& PointcloudComponentController::GetConfiguration() const
    {
        return m_config;
    }

    void PointcloudComponentConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudComponentConfig, AZ::ComponentConfig>()
                ->Version(1)
                ->Field("Editor Entity Id", &PointcloudComponentConfig::m_editorEntityId)
                ->Field("Point Size", &PointcloudComponentConfig::m_pointSize)
                ->Field("PointcloudAsset", &PointcloudComponentConfig::m_pointcloudAsset);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PointcloudComponentConfig>("PointcloudComponentConfig", "Config for PointcloudComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudComponentConfig::m_pointSize,
                        "Point Size",
                        "Size of the points in the pointcloud")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudComponentConfig::m_pointcloudAsset,
                        "Pointcloud Asset",
                        "Asset containing the pointcloud data");
            }
        }
    }

    void PointcloudComponentController::Reflect(AZ::ReflectContext* context)
    {
        PointcloudComponentConfig::Reflect(context);
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<PointcloudComponentController>()
                    ->Version(1)
                    ->Field("Configuration", &PointcloudComponentController::m_config)
                    ->Field("Point count", &PointcloudComponentController::m_pointCount);

                AZ::EditContext* editContext = serializeContext->GetEditContext();
                if (editContext)
                {
                    editContext->Class<PointcloudComponentController>("PointcloudComponentController", "PointcloudComponentController")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "PointcloudComponentController")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &PointcloudComponentController::m_config,
                            "Configuration",
                            "Configuration of the pointcloud")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudComponentController::OnAssetChanged)
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &PointcloudComponentController::m_pointCount,
                            "Point Count",
                            "Number of points in the pointcloud")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true);
                }
            }
        }
    }

    void PointcloudComponentController::Activate(AZ::EntityId entityId)
    {
        m_config.m_editorEntityId = entityId;
        AZ::TransformNotificationBus::Handler::BusConnect(m_config.m_editorEntityId);
        PointcloudComponentControllerConfigurationBus::Handler::BusConnect(m_config.m_editorEntityId);
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                m_scene = AZ::RPI::Scene::GetSceneForEntityId(m_config.m_editorEntityId);
                if (m_scene)
                {
                    m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();

                    AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
                    OnAssetChanged();
                }
            });
    }

    void PointcloudComponentController::Deactivate()
    {
        PointcloudComponentControllerConfigurationBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        if (m_featureProcessor)
        {
            m_featureProcessor->ReleasePointcloud(m_config.m_pointcloudHandle);
        }
    }

    AZ::Crc32 PointcloudComponentController::OnSetPointSize()
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->SetPointSize(m_config.m_pointcloudHandle, m_config.m_pointSize);
        }
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    AZ::Crc32 PointcloudComponentController::OnAssetChanged()
    {
        if (m_featureProcessor)
        {
            printf("Feature processor exists\n");
            m_featureProcessor->ReleasePointcloud(m_config.m_pointcloudHandle);
            if (m_config.m_pointcloudAsset.GetId().IsValid())
            {
                m_config.m_pointcloudHandle = m_featureProcessor->AcquirePointcloudFromAsset(m_config.m_pointcloudAsset);
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(
                    entity, &AZ::ComponentApplicationRequests::FindEntity, m_config.m_editorEntityId);
                m_featureProcessor->SetTransform(m_config.m_pointcloudHandle, entity->GetTransform()->GetWorldTM());
                m_featureProcessor->SetPointSize(m_config.m_pointcloudHandle, m_config.m_pointSize);
                m_pointCount = m_featureProcessor->GetPointCount(m_config.m_pointcloudHandle);
            }
        }
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    void PointcloudComponentController::OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible)
    {
        if (entityId == m_config.m_editorEntityId)
        {
            m_featureProcessor->SetVisibility(m_config.m_pointcloudHandle, visible);
        }
    }

    void PointcloudComponentController::OnTransformChanged([[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->SetTransform(m_config.m_pointcloudHandle, world);
        }
    }

    void PointcloudComponentController::SetPointcloudAsset(AZ::Data::Asset<PointcloudAsset> asset)
    {
        m_config.m_pointcloudAsset = asset;
    }
    void PointcloudComponentController::SetPointSize(float pointSize)
    {
        m_config.m_pointSize = pointSize;
    }

} // namespace Pointcloud
