
#include <RobotecRecordingTools/RobotecRecordingToolsTypeIds.h>
#include <RobotecRecordingToolsModuleInterface.h>
#include "RobotecRecordingToolsSystemComponent.h"

namespace RobotecRecordingTools
{
    class RobotecRecordingToolsModule
        : public RobotecRecordingToolsModuleInterface
    {
    public:
        AZ_RTTI(RobotecRecordingToolsModule, RobotecRecordingToolsModuleTypeId, RobotecRecordingToolsModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecRecordingToolsModule, AZ::SystemAllocator);
    };
}// namespace RobotecRecordingTools

AZ_DECLARE_MODULE_CLASS(Gem_RobotecRecordingTools, RobotecRecordingTools::RobotecRecordingToolsModule)
