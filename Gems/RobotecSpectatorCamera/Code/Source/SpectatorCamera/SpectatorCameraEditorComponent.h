#pragma once

#include "SpectatorCameraConfiguration.h"
#include <AzCore/Component/TransformBus.h>
#include <AzToolsFramework/API/EditorCameraBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

namespace RobotecSpectatorCamera
{
    class SpectatorCameraEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , public AZ::TransformNotificationBus::Handler
        , public Camera::EditorCameraNotificationBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(SpectatorCameraEditorComponent, SpectatorCameraEditorComponentTypeId);

        static void Reflect(AZ::ReflectContext* context);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void Activate() override;
        void Deactivate() override;

        void BuildGameEntity(AZ::Entity* gameEntity) override;

        // AZ::TransformNotificationBus::Handler overrides
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // Camera::EditorCameraNotificationBus::Handler overrides
        void OnViewportViewEntityChanged(const AZ::EntityId& newViewId) override;

    private:
        bool IsActiveViewCamera() const;
        //! Recompute the orbit radius from the entity's distance to the look-at point.
        void UpdateOrbitRadiusFromCurrentTransform();

        SpectatorCameraConfiguration m_configuration;
    };
} // namespace RobotecSpectatorCamera
