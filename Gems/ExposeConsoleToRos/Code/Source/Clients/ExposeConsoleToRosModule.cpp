
#include "ExposeConsoleToRosSystemComponent.h"
#include <ExposeConsoleToRos/ExposeConsoleToRosTypeIds.h>
#include <ExposeConsoleToRosModuleInterface.h>

namespace ExposeConsoleToRos
{
    class ExposeConsoleToRosModule : public ExposeConsoleToRosModuleInterface
    {
    public:
        AZ_RTTI(ExposeConsoleToRosModule, ExposeConsoleToRosModuleTypeId, ExposeConsoleToRosModuleInterface);
        AZ_CLASS_ALLOCATOR(ExposeConsoleToRosModule, AZ::SystemAllocator);
    };
} // namespace ExposeConsoleToRos

AZ_DECLARE_MODULE_CLASS(Gem_ExposeConsoleToRos, ExposeConsoleToRos::ExposeConsoleToRosModule)