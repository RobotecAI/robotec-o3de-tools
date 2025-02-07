
#include <TerrainShaper/TerrainShaperTypeIds.h>
#include <TerrainShaperModuleInterface.h>
#include "TerrainShaperSystemComponent.h"

namespace TerrainShaper
{
    class TerrainShaperModule
        : public TerrainShaperModuleInterface
    {
    public:
        AZ_RTTI(TerrainShaperModule, TerrainShaperModuleTypeId, TerrainShaperModuleInterface);
        AZ_CLASS_ALLOCATOR(TerrainShaperModule, AZ::SystemAllocator);
    };
}// namespace TerrainShaper

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), TerrainShaper::TerrainShaperModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_TerrainShaper, TerrainShaper::TerrainShaperModule)
#endif
