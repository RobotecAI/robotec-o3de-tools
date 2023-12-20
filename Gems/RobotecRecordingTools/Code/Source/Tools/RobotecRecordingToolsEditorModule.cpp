
#include <RobotecRecordingTools/RobotecRecordingToolsTypeIds.h>
#include <RobotecRecordingToolsModuleInterface.h>
#include "RobotecRecordingToolsEditorSystemComponent.h"

namespace RobotecRecordingTools
{
    class RobotecRecordingToolsEditorModule
        : public RobotecRecordingToolsModuleInterface
    {
    public:
        AZ_RTTI(RobotecRecordingToolsEditorModule, RobotecRecordingToolsEditorModuleTypeId, RobotecRecordingToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecRecordingToolsEditorModule, AZ::SystemAllocator);

        RobotecRecordingToolsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                RobotecRecordingToolsEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<RobotecRecordingToolsEditorSystemComponent>(),
            };
        }
    };
}// namespace RobotecRecordingTools

AZ_DECLARE_MODULE_CLASS(Gem_RobotecRecordingTools, RobotecRecordingTools::RobotecRecordingToolsEditorModule)
