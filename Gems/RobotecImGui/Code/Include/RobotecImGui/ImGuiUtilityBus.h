/*
 * Copyright (c) Robotec.ai 2023. All rights reserved.
 */

#pragma once

#include "AzCore/EBus/EBusSharedDispatchTraits.h"
#include "ImGuiElement.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/string/string.h>

namespace RobotecImGui
{
    //! Interface class that allows to add various ImGui callbacks
    //!
    //! Each function call can be processed without blocking Bus for other dispatches.
    //! Do not use connects / disconnects to this bus during event dispatch, as they are not allowed for this concurrency model.
    //! Those constraints allow for processing multiple ImGui requests in parallel.
    //! Bus implementations should allow for asynchronous execution of provided functions.
    class ImGuiRequests : public AZ::EBusSharedDispatchTraits<ImGuiRequests>
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        //! Get ImGui elements to be rendered
        virtual AZStd::vector<ImGuiElement> GetMessage() = 0;

        virtual AZStd::string GetEntityName() = 0;

    protected:
        ~ImGuiRequests() = default;
    };

    using ImGuiUtilityRequestBus = AZ::EBus<ImGuiRequests>;
} // namespace RobotecImGui
