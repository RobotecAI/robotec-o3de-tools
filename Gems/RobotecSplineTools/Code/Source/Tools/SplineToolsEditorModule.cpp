
#include "SplineToolsEditorComponent.h"
#include "SplineToolsEditorSystemComponent.h"
#include <SplineTools/SplineToolsTypeIds.h>
#include <SplineToolsModuleInterface.h>

namespace SplineTools
{
    class SplineToolsEditorModule : public SplineToolsModuleInterface
    {
    public:
        AZ_RTTI(SplineToolsEditorModule, SplineToolsEditorModuleTypeId, SplineToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(SplineToolsEditorModule, AZ::SystemAllocator);

        SplineToolsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    SplineToolsEditorSystemComponent::CreateDescriptor(),
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
                azrtti_typeid<SplineToolsEditorSystemComponent>(),
            };
        }
    };
} // namespace SplineTools

AZ_DECLARE_MODULE_CLASS(Gem_SplineTools, SplineTools::SplineToolsEditorModule)
