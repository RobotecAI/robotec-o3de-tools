#include "SmoothingComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/TransformBus.h>
#include "Utils.h"
void SmoothingConfig::Reflect(AZ::ReflectContext* context)
{

    if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
    {
        serializeContext->Class<SmoothingConfig, AZ::ComponentConfig>()
            ->Version(1)
            ->Field("m_entityToTrack", &SmoothingConfig::m_entityToTrack)
            ->Field("m_lockZAxis", &SmoothingConfig::m_lockZAxis);

        if (AZ::EditContext* editContext = serializeContext->GetEditContext())
        {
            editContext->Class<SmoothingConfig>("SmoothingConfig", "An example of a component that does something")
                ->DataElement(AZ::Edit::UIHandlers::Default, &SmoothingConfig::m_entityToTrack, "Entity to Track", "The entity to track for smoothing")
                ->DataElement(AZ::Edit::UIHandlers::Default, &SmoothingConfig::m_lockZAxis, "Lock Z Axis", "Lock the Z axis of the entity to track");
        }
    }
}
void SmoothingComponentController::Reflect(AZ::ReflectContext* context)
{
    SmoothingConfig::Reflect(context);
    if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
    {
        serializeContext->Class<SmoothingComponentController>()
            ->Field(
                "Configuration", &SmoothingComponentController::m_config)
            ->Version(1);
        if (AZ::EditContext* editContext = serializeContext->GetEditContext())
        {
            editContext->Class<SmoothingComponentController>("SmoothingComponentController", "")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                ->Attribute(AZ::Edit::Attributes::Category, "Smoothing")
                ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                ->DataElement(AZ::Edit::UIHandlers::Default, &SmoothingComponentController::m_config);
        }
    }
}

SmoothingComponentController::SmoothingComponentController(const SmoothingConfig& config)
    : m_config(config)
{
}


void SmoothingComponentController::Activate(AZ::EntityId entityId)
{
    m_entityId = entityId;
    AZ::TickBus::Handler::BusConnect();
}

void SmoothingComponentController::Deactivate()
{
    AZ::TickBus::Handler::BusDisconnect();
}

void SmoothingComponentController::OnTick(float deltaTime, AZ::ScriptTimePoint time)
{
    AZ_UNUSED(deltaTime);
    AZ_UNUSED(time);
    // get transform of entity to track
    AZ::Transform transform = AZ::Transform::CreateIdentity();
    AZ::TransformBus::EventResult(transform, m_config.m_entityToTrack, &AZ::TransformBus::Events::GetWorldTM);
    // lock Z axis if needed
    if (m_config.m_lockZAxis)
    {
        transform = SmoothingUtils::RemoveTiltFromTransform(transform);
    }
    // set transform to this entity
    AZ::TransformBus::Event(m_entityId, &AZ::TransformBus::Events::SetWorldTM, transform);

}

int SmoothingComponentController::GetTickOrder()
{
    return AZ::TICK_DEFAULT;
}

void SmoothingComponentController::Init()
{
}

void SmoothingComponentController::SetConfiguration(const SmoothingConfig& config)
{
    m_config = config;
}

const SmoothingConfig& SmoothingComponentController::GetConfiguration() const
{
    return m_config;
}

SmoothingComponent::SmoothingComponent(const SmoothingConfig& config)
    : SmoothingComponentBase(config)
{
}

void SmoothingComponent::Reflect(AZ::ReflectContext* context)
{
    SmoothingComponentBase::Reflect(context);
    if(auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
    {
        serializeContext->Class<SmoothingComponent, SmoothingComponentBase>()
            ->Version(1);
    }
}

void SmoothingComponent::Activate()
{
    SmoothingComponentBase::Activate();
}

void SmoothingComponent::Deactivate()
{
    SmoothingComponentBase::Deactivate();
}

