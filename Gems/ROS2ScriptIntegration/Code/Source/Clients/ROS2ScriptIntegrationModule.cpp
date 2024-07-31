
#include "ROS2ScriptIntegrationSystemComponent.h"
#include <ROS2ScriptIntegration/ROS2ScriptIntegrationTypeIds.h>
#include <ROS2ScriptIntegrationModuleInterface.h>

namespace ROS2ScriptIntegration
{
    class ROS2ScriptIntegrationModule : public ROS2ScriptIntegrationModuleInterface
    {
    public:
        AZ_RTTI(ROS2ScriptIntegrationModule, ROS2ScriptIntegrationModuleTypeId, ROS2ScriptIntegrationModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2ScriptIntegrationModule, AZ::SystemAllocator);
    };
} // namespace ROS2ScriptIntegration

AZ_DECLARE_MODULE_CLASS(Gem_ROS2ScriptIntegration, ROS2ScriptIntegration::ROS2ScriptIntegrationModule)
