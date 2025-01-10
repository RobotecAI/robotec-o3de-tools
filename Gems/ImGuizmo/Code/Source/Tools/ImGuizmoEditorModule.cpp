
#include <ImGuizmo/ImGuizmoTypeIds.h>
#include <ImGuizmoModuleInterface.h>
#include "ImGuizmoEditorSystemComponent.h"

namespace ImGuizmo
{
    class ImGuizmoEditorModule
        : public ImGuizmoModuleInterface
    {
    public:
        AZ_RTTI(ImGuizmoEditorModule, ImGuizmoEditorModuleTypeId, ImGuizmoModuleInterface);
        AZ_CLASS_ALLOCATOR(ImGuizmoEditorModule, AZ::SystemAllocator);

        ImGuizmoEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ImGuizmoEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<ImGuizmoEditorSystemComponent>(),
            };
        }
    };
}// namespace ImGuizmo

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ImGuizmo::ImGuizmoEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ImGuizmo_Editor, ImGuizmo::ImGuizmoEditorModule)
#endif
