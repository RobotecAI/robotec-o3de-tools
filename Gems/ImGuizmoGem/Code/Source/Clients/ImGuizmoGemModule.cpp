
#include <ImGuizmoGem/ImGuizmoGemTypeIds.h>
#include <ImGuizmoGemModuleInterface.h>
#include "ImGuizmoGemSystemComponent.h"

namespace ImGuizmoGem
{
    class ImGuizmoGemModule
        : public ImGuizmoGemModuleInterface
    {
    public:
        AZ_RTTI(ImGuizmoGemModule, ImGuizmoGemModuleTypeId, ImGuizmoGemModuleInterface);
        AZ_CLASS_ALLOCATOR(ImGuizmoGemModule, AZ::SystemAllocator);
    };
}// namespace ImGuizmoGem

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ImGuizmoGem::ImGuizmoGemModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ImGuizmoGem, ImGuizmoGem::ImGuizmoGemModule)
#endif
