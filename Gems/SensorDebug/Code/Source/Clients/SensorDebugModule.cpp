
#include <SensorDebug/SensorDebugTypeIds.h>
#include <SensorDebugModuleInterface.h>
#include "SensorDebugSystemComponent.h"

namespace SensorDebug
{
    class SensorDebugModule
        : public SensorDebugModuleInterface
    {
    public:
        AZ_RTTI(SensorDebugModule, SensorDebugModuleTypeId, SensorDebugModuleInterface);
        AZ_CLASS_ALLOCATOR(SensorDebugModule, AZ::SystemAllocator);
    };
}// namespace SensorDebug

AZ_DECLARE_MODULE_CLASS(Gem_SensorDebug, SensorDebug::SensorDebugModule)
