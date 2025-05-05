
#include "ImGuiProviderEditorSystemComponent.h"
#include <ImGuiProvider/ImGuiProviderTypeIds.h>
#include <ImGuiProviderModuleInterface.h>

namespace ImGuiProvider
{
    class ImGuiProviderEditorModule : public ImGuiProviderModuleInterface
    {
    public:
        AZ_RTTI(ImGuiProviderEditorModule, ImGuiProviderEditorModuleTypeId, ImGuiProviderModuleInterface);
        AZ_CLASS_ALLOCATOR(ImGuiProviderEditorModule, AZ::SystemAllocator);

        ImGuiProviderEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors
            // here. Add ALL components descriptors associated with this gem to
            // m_descriptors. This will associate the AzTypeInfo information for the
            // components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ImGuiProviderEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ImGuiProviderEditorSystemComponent>(),
            };
        }
    };
} // namespace ImGuiProvider

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ImGuiProvider::ImGuiProviderEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ImGuiProvider_Editor, ImGuiProvider::ImGuiProviderEditorModule)
#endif
