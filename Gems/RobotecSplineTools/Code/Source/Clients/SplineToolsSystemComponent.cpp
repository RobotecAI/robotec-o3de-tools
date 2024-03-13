
#include "SplineToolsSystemComponent.h"

#include <SplineTools/SplineToolsTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace SplineTools
{
    AZ_COMPONENT_IMPL(SplineToolsSystemComponent, "SplineToolsSystemComponent",
        SplineToolsSystemComponentTypeId);

    void SplineToolsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineToolsSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void SplineToolsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SplineToolsService"));
    }

    void SplineToolsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SplineToolsService"));
    }

    void SplineToolsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void SplineToolsSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    SplineToolsSystemComponent::SplineToolsSystemComponent()
    {
        if (SplineToolsInterface::Get() == nullptr)
        {
            SplineToolsInterface::Register(this);
        }
    }

    SplineToolsSystemComponent::~SplineToolsSystemComponent()
    {
        if (SplineToolsInterface::Get() == this)
        {
            SplineToolsInterface::Unregister(this);
        }
    }

    void SplineToolsSystemComponent::Init()
    {
    }

    void SplineToolsSystemComponent::Activate()
    {
        SplineToolsRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void SplineToolsSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        SplineToolsRequestBus::Handler::BusDisconnect();
    }

    void SplineToolsSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace SplineTools
