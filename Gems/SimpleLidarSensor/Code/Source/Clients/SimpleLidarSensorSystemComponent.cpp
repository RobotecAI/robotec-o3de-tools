#include "SimpleLidarSensorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>

namespace SimpleLidarSensor
{
    void SimpleLidarSensorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SimpleLidarSensorSystemComponent, AZ::Component>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SimpleLidarSensorSystemComponent>(
                      "Simple LIDAR Sensor System Component",
                      "System component responsible for setting up pass templates for Simple LIDAR sensor simulation.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                    ->Attribute(AZ::Edit::Attributes::Category, "SimpleLidarSensor")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void SimpleLidarSensorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("SimpleLidarSensorSystemService"));
    }

    void SimpleLidarSensorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("SimpleLidarSensorSystemService"));
    }

    void SimpleLidarSensorSystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // Temporarily removed ROS2Service dependency to isolate initialization issues
        // TODO: Re-add once basic system component loading is working
        // required.push_back(AZ_CRC("ROS2Service"));
    }

    void SimpleLidarSensorSystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC("RPISystem"));
    }

    SimpleLidarSensorSystemComponent::SimpleLidarSensorSystemComponent()
    {
    }

    SimpleLidarSensorSystemComponent::~SimpleLidarSensorSystemComponent()
    {
    }

    void SimpleLidarSensorSystemComponent::Init()
    {
    }

    void SimpleLidarSensorSystemComponent::Activate()
    {
        // Temporarily disabled pass template loading to resolve asset system initialization issues
        AZ::ApplicationTypeQuery appType;
        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
        if (appType.IsGame() || appType.IsEditor())
        {
            InitPassTemplateMappingsHandler();
        }

    }

    void SimpleLidarSensorSystemComponent::Deactivate()
    {
        m_loadTemplatesHandler.Disconnect();
    }

    void SimpleLidarSensorSystemComponent::InitPassTemplateMappingsHandler()
    {
        auto* passSystem = AZ::RPI::PassSystemInterface::Get();
        AZ_Assert(passSystem, "Cannot get the pass system.");

        m_loadTemplatesHandler = AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler(
            [this]()
            {
                this->LoadPassTemplateMappings();
            });
        passSystem->ConnectEvent(m_loadTemplatesHandler);
    }

    void SimpleLidarSensorSystemComponent::LoadPassTemplateMappings()
    {
        AZ_Printf("SimpleLidarSensorSystemComponent", "LoadPassTemplateMappings\n");
        auto* passSystem = AZ::RPI::PassSystemInterface::Get();
        AZ_Assert(passSystem, "PassSystemInterface is null");

        const char* passTemplatesFile = "Passes/LidarPassTemplates.azasset";
        [[maybe_unused]] const bool isOk = passSystem->LoadPassTemplateMappings(passTemplatesFile);
        AZ_Assert(isOk, "LoadPassTemplateMappings returned false");
    }

} // namespace SimpleLidarSensor