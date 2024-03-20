
#include "SplineToolsSystemComponent.h"
#include <SplineTools/SplineToolsTypeIds.h>
#include <SplineToolsModuleInterface.h>

namespace SplineTools
{
    class SplineToolsModule : public SplineToolsModuleInterface
    {
    public:
        AZ_RTTI(SplineToolsModule, SplineToolsModuleTypeId, SplineToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(SplineToolsModule, AZ::SystemAllocator);
    };
} // namespace SplineTools

AZ_DECLARE_MODULE_CLASS(Gem_SplineTools, SplineTools::SplineToolsModule)
