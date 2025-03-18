
#include "ImGuiProviderSystemComponent.h"
#include <ImGuiProvider/ImGuiProviderTypeIds.h>
#include <ImGuiProviderModuleInterface.h>

namespace ImGuiProvider
{
    class ImGuiProviderModule : public ImGuiProviderModuleInterface
    {
    public:
        AZ_RTTI(ImGuiProviderModule, ImGuiProviderModuleTypeId, ImGuiProviderModuleInterface);
        AZ_CLASS_ALLOCATOR(ImGuiProviderModule, AZ::SystemAllocator);
    };
} // namespace ImGuiProvider

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ImGuiProvider::ImGuiProviderModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ImGuiProvider, ImGuiProvider::ImGuiProviderModule)
#endif
