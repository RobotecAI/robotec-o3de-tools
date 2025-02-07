

#pragma once

#include "ROS2PoseControlConfiguration.h"
#include "ROS2PoseControlTypeIds.h"

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>

namespace ROS2PoseControl
{
    class ROS2PoseControlRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(ROS2PoseControlRequests, ROS2PoseControlRequestsTypeId)

        virtual void SetTrackingMode(const ROS2PoseControlConfiguration::TrackingMode trackingMode) = 0;
        virtual void SetTargetFrame(const AZStd::string& targetFrame) = 0;
        virtual void SetReferenceFrame(const AZStd::string& referenceFrame) = 0;
        virtual void SetEnablePhysics(bool enable) = 0;
        virtual void SetRigidBodiesToKinematic(bool enable)  = 0;
        virtual void ApplyConfiguration() = 0;
    };

    using ROS2PoseControlRequestsBus = AZ::EBus<ROS2PoseControlRequests>;
} // namespace ROS2PoseControl