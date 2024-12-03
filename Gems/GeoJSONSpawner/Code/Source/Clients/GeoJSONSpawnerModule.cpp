
#include "GeoJSONSpawnerSystemComponent.h"
#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>
#include <GeoJSONSpawnerModuleInterface.h>

namespace GeoJSONSpawner
{
    class GeoJSONSpawnerModule : public GeoJSONSpawnerModuleInterface
    {
    public:
        AZ_RTTI(GeoJSONSpawnerModule, GeoJSONSpawnerModuleTypeId, GeoJSONSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoJSONSpawnerModule, AZ::SystemAllocator);
    };
} // namespace GeoJSONSpawner

AZ_DECLARE_MODULE_CLASS(Gem_GeoJSONSpawner, GeoJSONSpawner::GeoJSONSpawnerModule)
