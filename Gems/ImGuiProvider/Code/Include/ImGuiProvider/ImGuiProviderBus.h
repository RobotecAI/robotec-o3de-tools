
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/base.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>
#include <ImGuiProvider/ImGuiProviderTypeIds.h>

#include <Atom/Feature/ImGui/SystemBus.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Interface/Interface.h>

namespace ImGuiProvider
{
    // path to feature on the toolbar e.g. "Tools1/Tools2/ActualFeature"
    using ImGuiFeaturePath = AZ::IO::Path;

    class ImGuiProviderRequests
    {
    public:
        AZ_RTTI(ImGuiProviderRequests, ImGuiProviderRequestsTypeId);
        virtual ~ImGuiProviderRequests() = default;

        //! @brief Retruns id of currently selected GUI if any is active, nullopt is no gu is active
        virtual AZStd::optional<ImGuiFeaturePath> GetActiveGuiId() = 0;

        //! @brief Activates GUI with given ID
        //! @param guiIdToActivate id of gui to activate. Id is set during registration;
        virtual void SetActiveGUI(const ImGuiFeaturePath& guiIdToActivate) = 0;
    };

    class ImGuiProviderBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ImGuiProviderRequestBus = AZ::EBus<ImGuiProviderRequests, ImGuiProviderBusTraits>;
    using ImGuiProviderInterface = AZ::Interface<ImGuiProviderRequests>;

    class ImGuiProviderNotifications : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(ImGuiProviderNotifications, ImGuiProviderNotificationsTypeId);
        virtual ~ImGuiProviderNotifications() = default;

        //! @brief Called to trigger ImGui update
        virtual void OnImGuiUpdate()
        {
        }

        //! @brief Called when ImGui registered by the handler was selected and is currently displayed.
        //! Called once per state change (hidden -> visible)
        virtual void OnImGuiSelected()
        {
        }

        //! @brief Called when ImGui registered by the handler was unselected and is currently hidden.
        //! Called once per state change (visible -> hidden)
        virtual void OnImGuiUnselected()
        {
        }
    };

    class ImGuiProviderNotificationBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        using BusIdType = ImGuiFeaturePath;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //////////////////////////////////////////////////////////////////////////
    };

    using ImGuiProviderNotificationBus = AZ::EBus<ImGuiProviderNotifications, ImGuiProviderNotificationBusTraits>;

} // namespace ImGuiProvider
