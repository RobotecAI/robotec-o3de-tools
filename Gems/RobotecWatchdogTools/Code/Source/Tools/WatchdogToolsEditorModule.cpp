
#include "WatchdogToolsEditorSystemComponent.h"
#include <WatchdogTools/WatchdogToolsTypeIds.h>
#include <WatchdogToolsModuleInterface.h>

namespace WatchdogTools
{
    class WatchdogToolsEditorModule : public WatchdogToolsModuleInterface
    {
    public:
        AZ_RTTI(WatchdogToolsEditorModule, WatchdogToolsEditorModuleTypeId, WatchdogToolsModuleInterface);

        AZ_CLASS_ALLOCATOR(WatchdogToolsEditorModule, AZ::SystemAllocator);

        WatchdogToolsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    WatchdogToolsEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<WatchdogToolsEditorSystemComponent>(),
            };
        }
    };
} // namespace WatchdogTools

AZ_DECLARE_MODULE_CLASS(Gem_WatchdogTools, WatchdogTools::WatchdogToolsEditorModule)
