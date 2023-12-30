
#include <RobotecImGui/RobotecImGuiTypeIds.h>
#include <RobotecImGuiModuleInterface.h>
#include "RobotecImGuiEditorSystemComponent.h"

namespace RobotecImGui
{
    class RobotecImGuiEditorModule
        : public RobotecImGuiModuleInterface
    {
    public:
        AZ_RTTI(RobotecImGuiEditorModule, RobotecImGuiEditorModuleTypeId, RobotecImGuiModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecImGuiEditorModule, AZ::SystemAllocator);

        RobotecImGuiEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                RobotecImGuiEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<RobotecImGuiEditorSystemComponent>(),
            };
        }
    };
}// namespace RobotecImGui

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), RobotecImGui::RobotecImGuiEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_RobotecImGui_Editor, RobotecImGui::RobotecImGuiEditorModule)
#endif
