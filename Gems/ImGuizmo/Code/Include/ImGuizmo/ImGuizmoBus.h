
#pragma once

#include <ImGuizmo/ImGuizmoTypeIds.h>

#include "ImGuizmo/ImGuizmoModes.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Transform.h>
namespace ImGuizmo
{

    class ImGuizmoRequests
    {
    public:
        AZ_RTTI(ImGuizmoRequests, ImGuizmoRequestsTypeId);
        virtual ~ImGuizmoRequests() = default;

        //! Get the transform of the gizmo
        virtual AZ::Transform GetGizmoTransform() = 0;

        //! Set the transform of the gizmo
        virtual void SetGizmoTransform(const AZ::Transform& transform) = 0;

        //! Set the gizmo visible
        //! @param visible true to make the gizmo visible, false to hide it
        virtual void SetGizmoVisible(bool visible) = 0;

        //! Set the gizmo operation, refer to ImGuizmo documentation for more information or
        //! ImGuizmoModes.h for the available operations
        //! @param operation the operation to set the gizmo to
        virtual void SetGizmoOperation(OPERATION operation) = 0;

        //! Set the gizmo mode
        //! @param mode the mode to set the gizmo to
        virtual void SetGizmoMode(MODE mode) = 0;

        //! Set the gizmo mode to local
        virtual void SetGizmoModeLocal()
        {
            SetGizmoMode(MODE::LOCAL);
        }

        //! Set the gizmo mode to world
        virtual void SetGizmoModeWorld()
        {
            SetGizmoMode(MODE::WORLD);
        }
    };

    class ImGuizmoBusTraits : public AZ::EBusTraits
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
