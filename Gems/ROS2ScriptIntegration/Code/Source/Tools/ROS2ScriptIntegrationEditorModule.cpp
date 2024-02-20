
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>
#include <ROS2ScriptIntegrationModuleInterface.h>
#include "ROS2ScriptIntegrationEditorSystemComponent.h"

namespace ROS2ScriptIntegration
{
    class ROS2ScriptIntegrationEditorModule
        : public ROS2ScriptIntegrationModuleInterface
    {
    public:
        AZ_RTTI(ROS2ScriptIntegrationEditorModule, ROS2ScriptIntegrationEditorModuleTypeId, ROS2ScriptIntegrationModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2ScriptIntegrationEditorModule, AZ::SystemAllocator);

        ROS2ScriptIntegrationEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ROS2ScriptIntegrationEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<ROS2ScriptIntegrationEditorSystemComponent>(),
            };
        }
    };
}// namespace ROS2ScriptIntegration

AZ_DECLARE_MODULE_CLASS(Gem_ROS2ScriptIntegration, ROS2ScriptIntegration::ROS2ScriptIntegrationEditorModule)
