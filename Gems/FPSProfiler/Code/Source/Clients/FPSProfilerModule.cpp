
#include <FPSProfiler/FPSProfilerTypeIds.h>
#include <FPSProfilerModuleInterface.h>
#include "FPSProfilerSystemComponent.h"

namespace FPSProfiler
{
    class FPSProfilerModule
        : public FPSProfilerModuleInterface
    {
    public:
        AZ_RTTI(FPSProfilerModule, FPSProfilerModuleTypeId, FPSProfilerModuleInterface);
        AZ_CLASS_ALLOCATOR(FPSProfilerModule, AZ::SystemAllocator);
    };
}// namespace FPSProfiler

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), FPSProfiler::FPSProfilerModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_FPSProfiler, FPSProfiler::FPSProfilerModule)
#endif
