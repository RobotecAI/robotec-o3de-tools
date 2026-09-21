/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <SplineTools/SplineToolsTypeIds.h>

namespace SplineTools
{
    //! Controls a SplineFollower component placed on the same entity.
    class SplineFollowerRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(SplineFollowerRequests, SplineFollowerRequestsTypeId);

        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        //! Begin following the spline. Progress is re-localized to the point on the spline
        //! nearest to the entity, so this is also the way to resume after a manual reposition.
        virtual void StartFollowing() = 0;

        //! Stop following and publish a zero velocity command so the robot comes to a halt.
        virtual void StopFollowing() = 0;

        //! @return True while velocity commands are being published.
        virtual bool IsFollowing() const = 0;

        //! @return Distance travelled along the spline, in meters from the spline's first vertex.
        virtual float GetDistanceAlongSpline() const = 0;

        //! @return Signed lateral distance to the spline in meters; positive when the spline is to the entity's left.
        virtual float GetCrossTrackError() const = 0;
    };

    using SplineFollowerRequestBus = AZ::EBus<SplineFollowerRequests>;

    class SplineFollowerNotifications : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(SplineFollowerNotifications, SplineFollowerNotificationsTypeId);

        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        //! Raised when the end of an open spline is reached. Closed (looping) splines never raise this.
        virtual void OnSplineFollowingFinished()
        {
        }
    };

    using SplineFollowerNotificationBus = AZ::EBus<SplineFollowerNotifications>;
} // namespace SplineTools
