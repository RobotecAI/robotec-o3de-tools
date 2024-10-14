
#include "ExposeConsoleToRosEditorSystemComponent.h"
#include <ExposeConsoleToRos/ExposeConsoleToRosTypeIds.h>
#include <ExposeConsoleToRosModuleInterface.h>

namespace ExposeConsoleToRos
{
    class ExposeConsoleToRosEditorModule : public ExposeConsoleToRosModuleInterface
    {
    public:
        AZ_RTTI(ExposeConsoleToRosEditorModule, ExposeConsoleToRosEditorModuleTypeId, ExposeConsoleToRosModuleInterface);
        AZ_CLASS_ALLOCATOR(ExposeConsoleToRosEditorModule, AZ::SystemAllocator);

        ExposeConsoleToRosEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ExposeConsoleToRosEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ExposeConsoleToRosEditorSystemComponent>(),
            };
        }
    };
} // namespace ExposeConsoleToRos

AZ_DECLARE_MODULE_CLASS(Gem_ExposeConsoleToRos_Editor, ExposeConsoleToRos::ExposeConsoleToRosEditorModule)
