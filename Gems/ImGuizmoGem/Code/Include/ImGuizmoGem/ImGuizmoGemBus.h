
#pragma once

#include <ImGuizmoGem/ImGuizmoGemTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ImGuizmoGem
{
    class ImGuizmoGemRequests
    {
    public:
        AZ_RTTI(ImGuizmoGemRequests, ImGuizmoGemRequestsTypeId);
        virtual ~ImGuizmoGemRequests() = default;
        // Put your public methods here
    };

    class ImGuizmoGemBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ImGuizmoGemRequestBus = AZ::EBus<ImGuizmoGemRequests, ImGuizmoGemBusTraits>;
    using ImGuizmoGemInterface = AZ::Interface<ImGuizmoGemRequests>;

} // namespace ImGuizmoGem
