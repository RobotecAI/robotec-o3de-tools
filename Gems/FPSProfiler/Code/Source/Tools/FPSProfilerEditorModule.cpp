#include "Clients/FPSProfilerComponent.h"

#include <FPSProfiler/FPSProfilerTypeIds.h>
#include <FPSProfilerModuleInterface.h>

namespace FPSProfiler
{
    class FPSProfilerEditorModule : public FPSProfilerModuleInterface
    {
    public:
        AZ_RTTI(FPSProfilerEditorModule, FPSProfilerEditorModuleTypeId, FPSProfilerModuleInterface);
        AZ_CLASS_ALLOCATOR(FPSProfilerEditorModule, AZ::SystemAllocator);

        FPSProfilerEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    FPSProfilerComponent::CreateDescriptor(),
                });
        }
    };
} // namespace FPSProfiler

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), FPSProfiler::FPSProfilerEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_FPSProfiler_Editor, FPSProfiler::FPSProfilerEditorModule)
#endif
