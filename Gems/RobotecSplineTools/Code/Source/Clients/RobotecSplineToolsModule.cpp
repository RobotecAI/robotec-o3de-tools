
#include <RobotecSplineTools/RobotecSplineToolsTypeIds.h>
#include <RobotecSplineToolsModuleInterface.h>
#include "RobotecSplineToolsSystemComponent.h"

namespace RobotecSplineTools
{
    class RobotecSplineToolsModule
        : public RobotecSplineToolsModuleInterface
    {
    public:
        AZ_RTTI(RobotecSplineToolsModule, RobotecSplineToolsModuleTypeId, RobotecSplineToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecSplineToolsModule, AZ::SystemAllocator);
    };
}// namespace RobotecSplineTools

AZ_DECLARE_MODULE_CLASS(Gem_RobotecSplineTools, RobotecSplineTools::RobotecSplineToolsModule)
