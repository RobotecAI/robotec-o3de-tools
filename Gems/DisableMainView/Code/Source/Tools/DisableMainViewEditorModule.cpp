
#include "DisableMainViewEditorSystemComponent.h"
#include <DisableMainView/DisableMainViewTypeIds.h>
#include <DisableMainViewModuleInterface.h>

namespace DisableMainView
{
    class DisableMainViewEditorModule : public DisableMainViewModuleInterface
    {
    public:
        AZ_RTTI(DisableMainViewEditorModule, DisableMainViewEditorModuleTypeId, DisableMainViewModuleInterface);

        AZ_CLASS_ALLOCATOR(DisableMainViewEditorModule, AZ::SystemAllocator);

        DisableMainViewEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    DisableMainViewEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<DisableMainViewEditorSystemComponent>(),
            };
        }
    };
} // namespace DisableMainView

AZ_DECLARE_MODULE_CLASS(Gem_DisableMainView, DisableMainView::DisableMainViewEditorModule)
