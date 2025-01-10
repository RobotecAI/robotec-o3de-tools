
#include "ImGuizmoSystemComponent.h"
#include <ImGuizmo/ImGuizmoTypeIds.h>
#include <ImGuizmoModuleInterface.h>

namespace ImGuizmo
{
    class ImGuizmoModule : public ImGuizmoModuleInterface
    {
    public:
        AZ_RTTI(ImGuizmoModule, ImGuizmoModuleTypeId, ImGuizmoModuleInterface);
        AZ_CLASS_ALLOCATOR(ImGuizmoModule, AZ::SystemAllocator);
    };
} // namespace ImGuizmo

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ImGuizmo::ImGuizmoModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ImGuizmo, ImGuizmo::ImGuizmoModule)
#endif
