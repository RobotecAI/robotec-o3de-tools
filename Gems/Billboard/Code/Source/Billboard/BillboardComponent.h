#pragma once

#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/containers/set.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <Billboard/BillboardTypeIds.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>

namespace BillboardComponent
{

    //! Component that adjust rotation of the entity to always face the given position, tag or entity
    //! In position strategy, the entity will always face the given position,
    //! in tag strategy, the entity will always face the mean position of the entities with the given tag,
    //! in entity strategy, the entity will always face the given entity
    class BillboardComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private LmbrCentral::TagGlobalNotificationBus::Handler
    {
    public:
        enum class Strategy
        {
            FaceLocation,
            FaceTag,
            FaceEntity
        };

        AZ_COMPONENT(BillboardComponent, BillboardComponentTypeId);

        BillboardComponent() = default;
        ~BillboardComponent() = default;

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void Reflect(AZ::ReflectContext* context);

        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

    private:
        AZ::Crc32 VisibilityLocation() const;
        AZ::Crc32 VisibilityTag() const;
        AZ::Crc32 VisibilityEntity() const;

        // AZ::TickBus::Handler overrides ...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // LmbrCentral::TagGlobalNotifications::Handler overrides
        void OnEntityTagAdded(const AZ::EntityId& entityId) override;
        void OnEntityTagRemoved(const AZ::EntityId& entityId) override;

        //! Look at the given position
        //! \param targetPosition The position to look at
        void LookAtZUp(const AZ::Vector3& targetPosition) const;

        //! Get the position to face taking into account the strategy
        //! \return The position to face
        AZ::Vector3 GetTargetPosition() const;

        AZ::Vector3 m_faceToBillboard = AZ::Vector3::CreateAxisX(); //! Local axis
        AZ::Matrix3x3 m_offsetRotation = AZ::Matrix3x3::CreateIdentity(); //! Offset rotation

        Strategy m_strategy = Strategy::FaceLocation; //! Strategy to use
        AZ::Vector3 m_scenePosition; //! Position to face - strategy FaceLocation
        AZ::EntityId m_targetEntityId; //! Entity to face - strategy FaceEntity
        AZStd::string m_targetTag; //! Tag to face - strategy FaceTag
        AZStd::unordered_set<AZ::EntityId> m_targetEntities; //! Entities with the tag to face - strategy FaceTag
    };
} // namespace BillboardComponent
