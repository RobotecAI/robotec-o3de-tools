
#pragma once

#include <ImGuizmo/ImGuizmoTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ImGuizmo
{
    class ImGuizmoRequests
    {
    public:
        AZ_RTTI(ImGuizmoRequests, ImGuizmoRequestsTypeId);
        virtual ~ImGuizmoRequests() = default;
        // Put your public methods here
    };

    class ImGuizmoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ImGuizmoRequestBus = AZ::EBus<ImGuizmoRequests, ImGuizmoBusTraits>;
    using ImGuizmoInterface = AZ::Interface<ImGuizmoRequests>;

} // namespace ImGuizmo
