/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Tools/Components/EditorPointcloudComponent.h>
#include <AzFramework/StringFunc/StringFunc.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/IO/SystemFile.h>

namespace Pointcloud
{
    void EditorPointcloudComponent::Reflect(AZ::ReflectContext* context)
    {
        BaseClass::Reflect(context);

        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorPointcloudComponent, BaseClass>()
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorPointcloudComponent>(
                    "Pointcloud", "The Pointcloud component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Graphics")
                        ->Attribute(AZ::Edit::Attributes::Icon, "Icons/Components/Component_Placeholder.svg")
                        ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Icons/Components/Viewport/Component_Placeholder.svg")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->Attribute(AZ::Edit::Attributes::HelpPageURL, "")
                    ;
            }
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->ConstantProperty(PointcloudEditorSystemComponentTypeId, BehaviorConstant(AZ::Uuid(PointcloudEditorSystemComponentTypeId)))
                ->Attribute(AZ::Script::Attributes::Module, "render")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Automation);
        }
    }

    EditorPointcloudComponent::EditorPointcloudComponent()
    {
    }

    EditorPointcloudComponent::EditorPointcloudComponent(const PointcloudComponentConfig& config)
        : BaseClass(config)
    {
    }

    void EditorPointcloudComponent::Activate()
    {
        BaseClass::Activate();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusConnect();

        AZ::u64 entityId = (AZ::u64)GetEntityId();
        m_controller.m_configuration.m_entityId = entityId;
    }

    void EditorPointcloudComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        BaseClass::Deactivate();
    }

    void EditorPointcloudComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_controller.m_featureProcessor)
        {
            return;
        }
    }

    void EditorPointcloudComponent::OnEntityVisibilityChanged(bool visibility)
    {

    }
}
