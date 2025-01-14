
#pragma once

#include <ImGuizmo/ImGuizmoTypeIds.h>

#include "ImGuizmo/ImGuizmoModes.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/string/string.h>

namespace ImGuizmo
{

    class ImGuizmoRequests
    {
    public:
        AZ_RTTI(ImGuizmoRequests, ImGuizmoRequestsTypeId);
        virtual ~ImGuizmoRequests() = default;

        using GizmoHandle = int32_t;

        static constexpr GizmoHandle InvalidGizmoHandle{ -1 };

        //! Creates the new gizmo
        //! @param transform the intial transform to set the gizmo to
        //! @return the handle of the gizmo
        virtual GizmoHandle AcquireHandle(const AZ::Transform& transform, const AZStd::string& name) = 0;

        //! Releases the gizmo
        //! @param handle the handle of the gizmo to be destroyed
        virtual void ReleaseHandle(GizmoHandle handle) = 0;

        //! Get the transform of the gizmo
        //! @param handle the handle of the gizmo to get the transform of
        //! @return the transform of the gizmo
        virtual AZ::Transform GetGizmoTransform(GizmoHandle handle) = 0;

        //! Set the transform of the gizmo
        //! @param handle the handle of the gizmo to set the visibility of
        //! @param transform the transform to set the gizmo to
        virtual void SetGizmoTransform(GizmoHandle handle, const AZ::Transform& transform) = 0;

        //! Set the gizmo visible
        //! @note only one gizmo can be visible at a time, setting a gizmo to visible will hide the previous gizmo
        //! @param handle the handle of the gizmo to set the visibility of
        //! @param visible true to make the gizmo visible, false to hide it
        virtual void SetGizmoVisible(GizmoHandle handle, bool visible) = 0;

        //! Get if gizmo visible
        //! @param handle the handle of the gizmo to set the visibility of
        virtual bool GetGizmoVisible(GizmoHandle handle) = 0;

        //! Sets the gizmo operation, refer to ImGuizmo documentation for more information or
        //! ImGuizmoModes.h for the available operations
        //! @param handle the handle of the gizmo to set the operation of
        //! @param operation the operation to set the gizmo to
        virtual void SetGizmoOperation(GizmoHandle handle, OPERATION operation) = 0;

        //! Gets the gizmo operation, refer to ImGuizmo documentation for more information or
        //! ImGuizmoModes.h for the available operations
        //! @param handle the handle of the gizmo to set the operation of
        //! @return the operation of the gizmo
        virtual OPERATION GetGizmoOperation(GizmoHandle handle) = 0;

        //! Sets the gizmo label
        //! @param handle the handle of the gizmo to set the operation of
        virtual void SetGizmoLabel(GizmoHandle handle, const AZStd::string& name) = 0;

        //! Gets the gizmo label
        //! @param handle the handle of the gizmo to set the operation of
        //! @return the label of the gizmo
        virtual AZStd::string GetGizmoLabel(GizmoHandle handle) = 0;

        //! Returns if the gizmo is being manipulated
        //! @param handle the handle of the gizmo to set the operation of
        //! @return true if the gizmo is being manipulated, false otherwise
        virtual bool GetIfManipulated(GizmoHandle handle) = 0;

        //! Set the gizmo mode
        //! @param mode the mode to set the gizmo to
        virtual void SetGizmoMode(GizmoHandle handle, MODE mode) = 0;

        //! Set the gizmo mode to local
        virtual void SetGizmoModeLocal(GizmoHandle handle)
        {
            SetGizmoMode(handle, MODE::LOCAL);
        }

        //! Set the gizmo mode to world
        virtual void SetGizmoModeWorld(GizmoHandle handle)
        {
            SetGizmoMode(handle, MODE::WORLD);
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
