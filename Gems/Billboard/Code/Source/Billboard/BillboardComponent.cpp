
#include "BillboardComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace BillboardComponent
{
    void BillboardComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService", 0x8ee22c50));
    }

    void BillboardComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<BillboardComponent, AZ::Component>()
                ->Version(0)
                ->Field("ScenePosition", &BillboardComponent::m_scenePosition)
                ->Field("Strategy", &BillboardComponent::m_strategy)
                ->Field("TargetEntityId", &BillboardComponent::m_targetEntityId)
                ->Field("TargetTag", &BillboardComponent::m_targetTag)
                ->Field("LocalFaceToBillboard", &BillboardComponent::m_faceToBillboard);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<BillboardComponent>("BillboardComponent", "Add a billboard to the entity")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecAgriculture")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &BillboardComponent::m_strategy, "Strategy", "Strategy to use")
                    ->EnumAttribute(Strategy::FaceLocation, "Face Location")
                    ->EnumAttribute(Strategy::FaceTag, "Face Tag")
                    ->EnumAttribute(Strategy::FaceEntity, "Face Entity")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &BillboardComponent::m_targetEntityId, "Target Entity", "Entity to face")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &BillboardComponent::VisibilityEntity)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &BillboardComponent::m_scenePosition, "Scene Position", "Position to face")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &BillboardComponent::VisibilityLocation)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &BillboardComponent::m_targetTag, "Target Tag", "Tag to face")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &BillboardComponent::VisibilityTag)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &BillboardComponent::m_faceToBillboard,
                        "Local Face To Billboard",
                        "Local face to billboard");
            }
        }
    }

    AZ::Crc32 BillboardComponent::VisibilityLocation() const
    {
        return m_strategy == Strategy::FaceLocation ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 BillboardComponent::VisibilityTag() const
    {
        return m_strategy == Strategy::FaceTag ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 BillboardComponent::VisibilityEntity() const
    {
        return m_strategy == Strategy::FaceEntity ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    void BillboardComponent::Activate()
    {
        m_faceToBillboard.NormalizeSafe();

        // compute local offset
        m_offsetRotation = AZ::Matrix3x3::CreateFromColumns(
            m_faceToBillboard, AZ::Vector3::CreateAxisZ().Cross(m_faceToBillboard), AZ::Vector3::CreateAxisZ());

        if (m_strategy == Strategy::FaceLocation)
        {
            LookAtZUp(m_scenePosition);
        }
        else if (m_strategy == Strategy::FaceEntity)
        {
            AZ::TickBus::Handler::BusConnect();
        }
        else if (m_strategy == Strategy::FaceTag)
        {
            const auto tag = LmbrCentral::Tag(m_targetTag);
            AZ::TickBus::Handler::BusConnect();
            LmbrCentral::TagGlobalNotificationBus::Handler::BusConnect(tag);
            // get the entities with the tag
            AZ::EBusAggregateResults<AZ::EntityId> aggregator;
            LmbrCentral::TagGlobalRequestBus::EventResult(aggregator, tag, &LmbrCentral::TagGlobalRequests::RequestTaggedEntities);
            m_targetEntities.insert(aggregator.values.begin(), aggregator.values.end());
        }
    }

    void BillboardComponent::Deactivate()
    {
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
        if (LmbrCentral::TagGlobalNotificationBus::Handler::BusIsConnected())
        {
            LmbrCentral::TagGlobalNotificationBus::Handler::BusDisconnect();
        }
    }

    void BillboardComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        LookAtZUp(GetTargetPosition());
    }

    void BillboardComponent::OnEntityTagAdded(const AZ::EntityId& entityId)
    {
        m_targetEntities.insert(entityId);
    }

    void BillboardComponent::OnEntityTagRemoved(const AZ::EntityId& entityId)
    {
        m_targetEntities.erase(entityId);
    }

    void BillboardComponent::LookAtZUp(const AZ::Vector3& targetPosition) const
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        AZ::Vector3 rayFromObjectToCamera = -transform.GetTranslation() + targetPosition;
        rayFromObjectToCamera.SetZ(0.0f); // we want to look at the target position in the XY plane (Z up)ush
        rayFromObjectToCamera.NormalizeSafe();

        const AZ::Vector3 up = AZ::Vector3::CreateAxisZ();
        const AZ::Vector3 right = up.Cross(rayFromObjectToCamera);

        const AZ::Matrix3x3 rotationMatrix = AZ::Matrix3x3::CreateFromColumns(rayFromObjectToCamera, right, up);
        AZ::Transform newTransform =
            AZ::Transform::CreateFromMatrix3x3AndTranslation(rotationMatrix * m_offsetRotation, transform.GetTranslation());

        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, newTransform);
    }

    AZ::Vector3 BillboardComponent::GetTargetPosition() const
    {
        if (m_strategy == Strategy::FaceLocation)
        {
            return m_scenePosition;
        }
        else if (m_strategy == Strategy::FaceEntity)
        {
            AZ::Vector3 targetPosition = AZ::Vector3::CreateZero();
            AZ::TransformBus::EventResult(targetPosition, m_targetEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
            return targetPosition;
        }
        else if (m_strategy == Strategy::FaceTag)
        {
            AZ::Vector3 meanPosition = AZ::Vector3::CreateZero();
            for (const auto& entityId : m_targetEntities)
            {
                AZ::Vector3 targetPosition = AZ::Vector3::CreateZero();
                AZ::TransformBus::EventResult(targetPosition, entityId, &AZ::TransformBus::Events::GetWorldTranslation);
                meanPosition += targetPosition;
            }
            meanPosition /= m_targetEntities.size();
            return meanPosition;
        }
        return AZ::Vector3::CreateZero();
    }

} // namespace BillboardComponent
