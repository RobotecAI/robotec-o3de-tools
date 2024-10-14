
#include "SmoothingEditorComponent.h"
#include "SmoothingEditorSystemComponent.h"
#include <Smoothing/SmoothingTypeIds.h>
#include <SmoothingModuleInterface.h>
namespace Smoothing
{
    class SmoothingEditorModule : public SmoothingModuleInterface
    {
    public:
        AZ_RTTI(SmoothingEditorModule, SmoothingEditorModuleTypeId, SmoothingModuleInterface);
        AZ_CLASS_ALLOCATOR(SmoothingEditorModule, AZ::SystemAllocator);

        SmoothingEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    SmoothingEditorSystemComponent::CreateDescriptor(),
                    SmoothingComponentEditorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<SmoothingEditorSystemComponent>(),
            };
        }
    };
} // namespace Smoothing

AZ_DECLARE_MODULE_CLASS(Gem_Smoothing_Editor, Smoothing::SmoothingEditorModule)
