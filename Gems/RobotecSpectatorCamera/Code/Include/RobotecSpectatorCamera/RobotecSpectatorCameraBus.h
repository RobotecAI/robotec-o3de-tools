
#pragma once

#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RobotecSpectatorCamera
{
    class RobotecSpectatorCameraRequests
    {
    public:
        AZ_RTTI(RobotecSpectatorCameraRequests, RobotecSpectatorCameraRequestsTypeId);
        virtual ~RobotecSpectatorCameraRequests() = default;
        // Put your public methods here
    };

    class RobotecSpectatorCameraBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RobotecSpectatorCameraRequestBus = AZ::EBus<RobotecSpectatorCameraRequests, RobotecSpectatorCameraBusTraits>;
    using RobotecSpectatorCameraInterface = AZ::Interface<RobotecSpectatorCameraRequests>;

} // namespace RobotecSpectatorCamera
