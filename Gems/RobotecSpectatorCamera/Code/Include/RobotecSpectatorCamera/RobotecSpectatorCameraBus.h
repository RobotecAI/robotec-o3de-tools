
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

        virtual CameraMode GetCameraMode() const = 0;
        virtual void SetCameraMode(const CameraMode cameraMode) = 0;

        virtual float GetMouseSensitivity() const = 0;
        virtual void SetMouseSensitivity(const float mouseSensitivity) = 0;

        virtual float GetCameraSpeed() const = 0;
        virtual void SetCameraSpeed(const float cameraSpeed) = 0;

        virtual bool GetFollowTargetRotation() const = 0;
        virtual void SetFollowTargetRotation(const bool followTargetRotation) = 0;

        virtual float GetVerticalOffset() const = 0;
        virtual void SetVerticalOffset(const float verticalOffset) = 0;
    };

    using RobotecSpectatorCameraRequestBus = AZ::EBus<RobotecSpectatorCameraRequests>;
} // namespace RobotecSpectatorCamera
