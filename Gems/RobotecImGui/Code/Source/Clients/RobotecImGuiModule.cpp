
#include <RobotecImGui/RobotecImGuiTypeIds.h>
#include <RobotecImGuiModuleInterface.h>
#include "RobotecImGuiSystemComponent.h"

namespace RobotecImGui
{
    class RobotecImGuiModule
        : public RobotecImGuiModuleInterface
    {
    public:
        AZ_RTTI(RobotecImGuiModule, RobotecImGuiModuleTypeId, RobotecImGuiModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecImGuiModule, AZ::SystemAllocator);
    };
}// namespace RobotecImGui

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), RobotecImGui::RobotecImGuiModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_RobotecImGui, RobotecImGui::RobotecImGuiModule)
#endif
