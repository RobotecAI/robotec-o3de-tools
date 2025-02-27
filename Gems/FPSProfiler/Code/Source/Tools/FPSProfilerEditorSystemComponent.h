#pragma once

#include "FPSProfilerConfig.h"
#include <Clients/FPSProfilerSystemComponent.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <ToolsComponents/EditorComponentBase.h>

namespace FPSProfiler
{
    /// System component for FPSProfiler editor
    class FPSProfilerEditorSystemComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(FPSProfilerEditorSystemComponent, FPSProfilerEditorSystemComponentTypeId, EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        FPSProfilerEditorSystemComponent() = default;
        ~FPSProfilerEditorSystemComponent() override = default;

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity*) override;

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        FPSProfilerConfig m_configuration;
    };
} // namespace FPSProfiler
