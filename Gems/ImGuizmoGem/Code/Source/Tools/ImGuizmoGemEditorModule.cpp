
#include <ImGuizmoGem/ImGuizmoGemTypeIds.h>
#include <ImGuizmoGemModuleInterface.h>
#include "ImGuizmoGemEditorSystemComponent.h"

namespace ImGuizmoGem
{
    class ImGuizmoGemEditorModule
        : public ImGuizmoGemModuleInterface
    {
    public:
        AZ_RTTI(ImGuizmoGemEditorModule, ImGuizmoGemEditorModuleTypeId, ImGuizmoGemModuleInterface);
        AZ_CLASS_ALLOCATOR(ImGuizmoGemEditorModule, AZ::SystemAllocator);

        ImGuizmoGemEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ImGuizmoGemEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<ImGuizmoGemEditorSystemComponent>(),
            };
        }
    };
}// namespace ImGuizmoGem

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ImGuizmoGem::ImGuizmoGemEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ImGuizmoGem_Editor, ImGuizmoGem::ImGuizmoGemEditorModule)
#endif
