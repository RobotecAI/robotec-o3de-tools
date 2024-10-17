
#pragma once

#include <PhysicalTimeTools/PhysicalTimeToolsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace PhysicalTimeTools
{
    class PhysicalTimeToolsRequests
    {
    public:
        AZ_RTTI(PhysicalTimeToolsRequests, PhysicalTimeToolsRequestsTypeId);
        virtual ~PhysicalTimeToolsRequests() = default;
//        virtual void Pause(bool isPaused) = 0;
//        virtual bool IsPaused() = 0;
//        virtual void SetTimeScale(float timeScale) = 0;
//        virtual float GetTimeScale() = 0;
    };

    class PhysicalTimeToolsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using PhysicalTimeToolsRequestBus = AZ::EBus<PhysicalTimeToolsRequests, PhysicalTimeToolsBusTraits>;
    using PhysicalTimeToolsInterface = AZ::Interface<PhysicalTimeToolsRequests>;

} // namespace PhysicalTimeTools
