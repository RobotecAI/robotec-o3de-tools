
#include <PhysicalTimeTools/PhysicalTimeToolsTypeIds.h>
#include <PhysicalTimeToolsModuleInterface.h>
#include "PhysicalTimeToolsSystemComponent.h"

namespace PhysicalTimeTools
{
    class PhysicalTimeToolsModule
        : public PhysicalTimeToolsModuleInterface
    {
    public:
        AZ_RTTI(PhysicalTimeToolsModule, PhysicalTimeToolsModuleTypeId, PhysicalTimeToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(PhysicalTimeToolsModule, AZ::SystemAllocator);
    };
}// namespace PhysicalTimeTools

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), PhysicalTimeTools::PhysicalTimeToolsModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_PhysicalTimeTools, PhysicalTimeTools::PhysicalTimeToolsModule)
#endif
