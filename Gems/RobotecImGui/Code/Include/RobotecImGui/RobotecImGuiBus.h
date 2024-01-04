
#pragma once

#include <RobotecImGui/RobotecImGuiTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RobotecImGui
{
    class RobotecImGuiRequests
    {
    public:
        AZ_RTTI(RobotecImGuiRequests, RobotecImGuiRequestsTypeId);
        virtual ~RobotecImGuiRequests() = default;
        // Put your public methods here
    };

    class RobotecImGuiBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RobotecImGuiRequestBus = AZ::EBus<RobotecImGuiRequests, RobotecImGuiBusTraits>;
    using RobotecImGuiInterface = AZ::Interface<RobotecImGuiRequests>;

} // namespace RobotecImGui
