
#include "RobotecSplineToolsEditorSystemComponent.h"
#include "SplineToolsEditorComponent.h"
#include <RobotecSplineTools/RobotecSplineToolsTypeIds.h>
#include <RobotecSplineToolsModuleInterface.h>

namespace RobotecSplineTools
{
    class RobotecSplineToolsEditorModule : public RobotecSplineToolsModuleInterface
    {
    public:
        AZ_RTTI(RobotecSplineToolsEditorModule, RobotecSplineToolsEditorModuleTypeId, RobotecSplineToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecSplineToolsEditorModule, AZ::SystemAllocator);

        RobotecSplineToolsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    RobotecSplineToolsEditorSystemComponent::CreateDescriptor(),
                    SplineToolsEditorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RobotecSplineToolsEditorSystemComponent>(),
            };
        }
    };
} // namespace RobotecSplineTools

AZ_DECLARE_MODULE_CLASS(Gem_RobotecSplineTools, RobotecSplineTools::RobotecSplineToolsEditorModule)
