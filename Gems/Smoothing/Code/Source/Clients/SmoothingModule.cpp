
#include "SmoothingSystemComponent.h"
#include <Smoothing/SmoothingTypeIds.h>
#include <SmoothingModuleInterface.h>

namespace Smoothing
{
    class SmoothingModule : public SmoothingModuleInterface
    {
    public:
        AZ_RTTI(SmoothingModule, SmoothingModuleTypeId, SmoothingModuleInterface);
        AZ_CLASS_ALLOCATOR(SmoothingModule, AZ::SystemAllocator);
    };
} // namespace Smoothing

AZ_DECLARE_MODULE_CLASS(Gem_Smoothing, Smoothing::SmoothingModule)
