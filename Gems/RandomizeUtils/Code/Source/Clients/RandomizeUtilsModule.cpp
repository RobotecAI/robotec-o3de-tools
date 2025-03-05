
#include <RandomizeUtils/RandomizeUtilsTypeIds.h>
#include <RandomizeUtilsModuleInterface.h>
namespace RandomizeUtils
{
    class RandomizeUtilsModule : public RandomizeUtilsModuleInterface
    {
    public:
        AZ_RTTI(RandomizeUtilsModule, RandomizeUtilsModuleTypeId, RandomizeUtilsModuleInterface);
        AZ_CLASS_ALLOCATOR(RandomizeUtilsModule, AZ::SystemAllocator);
    };
} // namespace RandomizeUtils

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), RandomizeUtils::RandomizeUtilsModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_RandomizeUtils, RandomizeUtils::RandomizeUtilsModule)
#endif
