#pragma once

#include "SpectatorCameraConfiguration.h"
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>

namespace RobotecSpectatorCamera
{
    class SpectatorCameraEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(SpectatorCameraEditorComponent, SpectatorCameraEditorComponentTypeId);

        static void Reflect(AZ::ReflectContext* context);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        SpectatorCameraConfiguration m_configuration;
    };
} // namespace RobotecSpectatorCamera
