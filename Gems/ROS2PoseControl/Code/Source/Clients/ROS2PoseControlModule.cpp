
#include <ROS2PoseControl/ROS2PoseControlTypeIds.h>
#include <ROS2PoseControlModuleInterface.h>
#include "ROS2PoseControlSystemComponent.h"

namespace ROS2PoseControl
{
    class ROS2PoseControlModule
        : public ROS2PoseControlModuleInterface
    {
    public:
        AZ_RTTI(ROS2PoseControlModule, ROS2PoseControlModuleTypeId, ROS2PoseControlModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2PoseControlModule, AZ::SystemAllocator);
    };
}// namespace ROS2PoseControl

AZ_DECLARE_MODULE_CLASS(Gem_ROS2PoseControl, ROS2PoseControl::ROS2PoseControlModule)
