#include "SimpleLidar.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>


namespace SimpleLidarSensor
{
    // Reflect for serialization and scripting
    void SimpleLidar::Reflect(AZ::ReflectContext* context)
    {
        if (auto serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SimpleLidar, AZ::Component>()
            ->Version(1)
            ->Field("value", &SimpleLidar::m_value);


            if (auto ec = serialize->GetEditContext())
            {
                ec->Class<SimpleLidar>("SimpleLidar", "SimpleLidar using graphics pipeline")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"));
            }
        }

    }


    void SimpleLidar::Activate()
    {
        AZ_Printf("ExampleComponent", "Activated with value: %f", m_value);
    }


    void SimpleLidar::Deactivate()
    {
        AZ_Printf("ExampleComponent", "Deactivated");
    }


    void SimpleLidar::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("SimpleLidar"));
    }


    void SimpleLidar::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("SimpleLidar"));
    }


    void SimpleLidar::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // required.push_back(AZ_CRC("SomeOtherService"));
    }



}