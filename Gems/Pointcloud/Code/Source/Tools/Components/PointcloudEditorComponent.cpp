#include "PointcloudEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include <3rd/happly.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Entity/EntityContext.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Scene/Scene.h>
#include <AzFramework/Scene/SceneSystemInterface.h>
#include <QFileDialog>
#include <QMessageBox>

#include <AzFramework/Scene/SceneSystemInterface.h>

#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Scene/SceneSystemInterface.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/UI/UICore/WidgetHelpers.h>
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
                ->Field("Visible", &PointcloudEditorComponent::m_visible);
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
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::OnSetPointSize)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudEditorComponent::m_visible,
                        "Visible",
                        "Visibility of the pointcloud")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::OnVisibility)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PointcloudEditorComponent::m_pointcloudAsset,
                        "Pointcloud Asset",
                        "Asset containing the pointcloud data")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudEditorComponent::LoadCloud);
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
                if (m_scene && m_pointcloudAsset)
                {
                    m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();
                    AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
                    m_pointcloudAsset.QueueLoad();
                    m_pointcloudAsset.BlockUntilLoadComplete();

                    AZStd::vector<AZStd::vector<PointcloudAsset::CloudVertex>> cloudVertexDataChunks;

                    m_pointcloudHandle = m_featureProcessor->AcquirePointcloud(m_pointcloudAsset->m_data);

                    if (m_pointcloudHandle != PointcloudFeatureProcessorInterface::InvalidPointcloudHandle)
                    {
                        m_featureProcessor->SetTransform(m_pointcloudHandle, m_entity->GetTransform()->GetWorldTM());
                        m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
                    }
                }
            });
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void PointcloudEditorComponent::Deactivate()
    {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        m_featureProcessor->ReleasePointcloud(m_pointcloudHandle);
    }

    void PointcloudEditorComponent::BuildGameEntity([[maybe_unused]] AZ::Entity* gameEntity)
    {
    }

    AZ::Crc32 PointcloudEditorComponent::OnSetPointSize()
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
        }
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    AZ::Crc32 PointcloudEditorComponent::OnVisibility()
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->SetVisibility(m_pointcloudHandle, m_visible);
        }
        return AZ::Edit::PropertyRefreshLevels::None;
    }

    AZ::Crc32 PointcloudEditorComponent::LoadCloud()
    {
        if (m_pointcloudHandle != PointcloudFeatureProcessorInterface::InvalidPointcloudHandle)
        {
            m_featureProcessor->ReleasePointcloud(m_pointcloudHandle);
        }
        if (m_scene && m_pointcloudAsset)
        {
            m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();
            AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");
            m_pointcloudAsset.QueueLoad();
            m_pointcloudAsset.BlockUntilLoadComplete();

            m_pointcloudHandle = m_featureProcessor->AcquirePointcloud(m_pointcloudAsset->m_data);

            if (m_pointcloudHandle != PointcloudFeatureProcessorInterface::InvalidPointcloudHandle)
            {
                m_featureProcessor->SetTransform(m_pointcloudHandle, m_entity->GetTransform()->GetWorldTM());
                m_featureProcessor->SetPointSize(m_pointcloudHandle, m_pointSize);
            }
        }

        return AZ::Edit::PropertyRefreshLevels::None;
    }

    void PointcloudEditorComponent::OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world)
    {
        if (m_featureProcessor)
        {
            m_featureProcessor->SetTransform(m_pointcloudHandle, world);
        }
    }
} // namespace Pointcloud
