
#pragma once

#include "FPSProfilerData.h"
#include <Clients/FPSProfilerSystemComponent.h>

#include <ToolsComponents/EditorComponentBase.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

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

        FPSProfilerEditorSystemComponent();
        ~FPSProfilerEditorSystemComponent();

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity*) override;

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        FPSProfilerData m_configuration;
    };
} // namespace FPSProfiler
