/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders. If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include "ROS2PoseControlTypeIds.h"

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>

namespace ROS2PoseControl
{
    enum class TrackingMode
    {
        PoseMessages,
        TF2
    };

    //! Interface for the ROS2PoseControl
    //! Used for configuring the ROS2PoseControl at runtime
    class ROS2PoseControlRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(ROS2PoseControlRequests, ROS2PoseControlRequestsTypeId)

        //! Set the tracking mode
        //! @param trackingMode - new tracking mode, to check available tracking modes @see ROS2PoseControlConfigruation
        virtual void SetTrackingMode(const TrackingMode trackingMode) = 0;

        //! Set the target frame that is used in the TF2 tracking mode
        //! @param targetFrame - name of the frame that will be used as a target frame in a lookupTransform method
        virtual void SetTargetFrame(const AZStd::string& targetFrame) = 0;

        //! Set the reference frame that is used in the TF2 tracking mode
        //! @param referenceFrame - name of the frame that will be used as a reference (source) frame in a lookupTransform method
        virtual void SetReferenceFrame(const AZStd::string& referenceFrame) = 0;

        //! Enable or disable physics for all prefab's rigid bodies
        //! @param enable - enable/disable flag
        virtual void SetEnablePhysics(bool enable) = 0;

        //! Change the prefab's rigid bodies to either Kinematic (enable==true) or Simulated (enable==false)
        //! @param enable - enable/disable flag
        virtual void SetRigidBodiesToKinematic(bool enable) = 0;

        //! Apply configuration of the ROS2PoseControl in its current state. In general this function reinitialize the ROS2 intestines of
        //! the ROS2PoseControl
        virtual void ApplyConfiguration() = 0;
    };

    using ROS2PoseControlRequestsBus = AZ::EBus<ROS2PoseControlRequests>;
} // namespace ROS2PoseControl
