/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/Feature/Utils/EditorRenderComponentAdapter.h>

#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <Components/PointcloudComponent.h>

#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{
    inline constexpr AZ::TypeId EditorComponentTypeId { "{2F9DD96F-7948-43B6-9E63-6F2267E7B880}" };

    class EditorPointcloudComponent final
        : public AZ::Render::EditorRenderComponentAdapter<PointcloudComponentController, PointcloudComponent, PointcloudComponentConfig>
        , private AzToolsFramework::EditorComponentSelectionRequestsBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private AZ::TickBus::Handler
        , private AzToolsFramework::EditorEntityInfoNotificationBus::Handler
    {
    public:
        using BaseClass = AZ::Render::EditorRenderComponentAdapter <PointcloudComponentController, PointcloudComponent, PointcloudComponentConfig>;
        AZ_EDITOR_COMPONENT(EditorPointcloudComponent, EditorComponentTypeId, BaseClass);

        static void Reflect(AZ::ReflectContext* context);

        EditorPointcloudComponent();
        EditorPointcloudComponent(const PointcloudComponentConfig& config);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:

        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;


    };
}
