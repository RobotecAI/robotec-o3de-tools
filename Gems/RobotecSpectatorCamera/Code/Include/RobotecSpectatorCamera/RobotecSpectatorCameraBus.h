
#pragma once

#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <SpectatorCamera/SpectatorCameraConfiguration.h>

namespace RobotecSpectatorCamera
{
    class RobotecSpectatorCameraRequests : public AZ::ComponentBus
    {
    public:
        using BusIdType = AZ::EntityId;

        AZ_RTTI(RobotecSpectatorCameraRequests, RobotecSpectatorCameraRequestsTypeId);
        virtual ~RobotecSpectatorCameraRequests() = default;
        // Put your public methods here

        //! Get the current mode of the camera
        //! @return current camera mode
        virtual CameraMode GetCameraMode() const = 0;

        //! Set the camera mode
        //! @param cameraMode new camera mode
        virtual void SetCameraMode(const CameraMode cameraMode) = 0;

        //! Get the mouse sensitivity
        //! @return mouse sensitivity
        virtual float GetMouseSensitivity() const = 0;

        //! Set the mouse sensitivity
        //! @param mouseSensitivity new mouse sensitivity
        virtual void SetMouseSensitivity(const float mouseSensitivity) = 0;

        //! Get the camera speed
        //! @return camera speed
        virtual float GetCameraSpeed() const = 0;

        //! Set the camera speed
        //! @param cameraSpeed new camera speed
        virtual void SetCameraSpeed(const float cameraSpeed) = 0;

        //! Get the value of the follow target rotation flag
        //! @return value of the follow target rotation flag
        virtual bool GetFollowTargetRotation() const = 0;

        //! Set the value of the follow target rotation flag
        //! @param followTargetRotation new value of the follow target rotation flag
        virtual void SetFollowTargetRotation(const bool followTargetRotation) = 0;

        //! Get the vertical offset
        //! This offset is used to change (in Z-Axis) the point around which the cam orbits
        //! @return vertical offset
        virtual float GetVerticalOffset() const = 0;

        //! Set the vertical offset
        //! This offset is used to change (in Z-Axis) the point around which the cam orbits
        //! @param verticalOffset new vertical offset
        virtual void SetVerticalOffset(const float verticalOffset) = 0;
    };

    using RobotecSpectatorCameraRequestBus = AZ::EBus<RobotecSpectatorCameraRequests>;
} // namespace RobotecSpectatorCamera
