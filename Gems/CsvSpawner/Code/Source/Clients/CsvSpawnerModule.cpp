
#include <CsvSpawner/CsvSpawnerTypeIds.h>
#include <CsvSpawnerModuleInterface.h>

namespace CsvSpawner
{
    class CsvSpawnerModule : public CsvSpawnerModuleInterface
    {
    public:
        AZ_RTTI(CsvSpawnerModule, CsvSpawnerModuleTypeId, CsvSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(CsvSpawnerModule, AZ::SystemAllocator);
    };
} // namespace CsvSpawner

AZ_DECLARE_MODULE_CLASS(Gem_CsvSpawner, CsvSpawner::CsvSpawnerModule)
