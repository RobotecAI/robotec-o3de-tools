
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/FPSProfilerSystemComponent.h>

namespace FPSProfiler
{
    /// System component for FPSProfiler editor
    class FPSProfilerEditorSystemComponent
        : public FPSProfilerSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = FPSProfilerSystemComponent;
    public:
        AZ_COMPONENT_DECL(FPSProfilerEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        FPSProfilerEditorSystemComponent();
        ~FPSProfilerEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        AZ::IO::Path m_OutputFilename = "@user@/fps_log.csv";
        bool m_SaveFPSData = true;
        bool m_SaveMultiple = true;
        bool m_SaveGPUData = true;
        bool m_SaveCPUData = true;
        bool m_ShowFPS = true;
    };
} // namespace FPSProfiler
