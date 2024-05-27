
#include <ROS2PoseControl/ROS2PoseControlTypeIds.h>
#include <ROS2PoseControlModuleInterface.h>
#include "ROS2PoseControlEditorSystemComponent.h"

namespace ROS2PoseControl
{
    class ROS2PoseControlEditorModule
        : public ROS2PoseControlModuleInterface
    {
    public:
        AZ_RTTI(ROS2PoseControlEditorModule, ROS2PoseControlEditorModuleTypeId, ROS2PoseControlModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2PoseControlEditorModule, AZ::SystemAllocator);

        ROS2PoseControlEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ROS2PoseControlEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<ROS2PoseControlEditorSystemComponent>(),
            };
        }
    };
}// namespace ROS2PoseControl

AZ_DECLARE_MODULE_CLASS(Gem_ROS2PoseControl, ROS2PoseControl::ROS2PoseControlEditorModule)
