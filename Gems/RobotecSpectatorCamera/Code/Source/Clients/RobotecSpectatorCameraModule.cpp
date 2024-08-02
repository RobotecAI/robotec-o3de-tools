
#include "RobotecSpectatorCameraSystemComponent.h"
#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>
#include <RobotecSpectatorCameraModuleInterface.h>

namespace RobotecSpectatorCamera
{
    class RobotecSpectatorCameraModule : public RobotecSpectatorCameraModuleInterface
    {
    public:
        AZ_RTTI(RobotecSpectatorCameraModule, RobotecSpectatorCameraModuleTypeId, RobotecSpectatorCameraModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecSpectatorCameraModule, AZ::SystemAllocator);
    };
} // namespace RobotecSpectatorCamera

AZ_DECLARE_MODULE_CLASS(Gem_RobotecSpectatorCamera, RobotecSpectatorCamera::RobotecSpectatorCameraModule)
