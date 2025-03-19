#pragma once

#include <Configurations/FPSProfilerConfig.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <ToolsComponents/EditorComponentBase.h>

namespace FPSProfiler
{
    /// System component for FPSProfiler editor
    class FPSProfilerEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(FPSProfilerEditorComponent, FPSProfilerEditorComponentTypeId, EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        FPSProfilerEditorComponent() = default;
        ~FPSProfilerEditorComponent() override = default;

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity*) override;

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        Configs::FileSaveSettings m_configFile; //!< Stores editor settings for the profiler
        Configs::RecordSettings m_configRecord; //!< Stores editor settings for the profiler
        Configs::PrecisionSettings m_configPrecision; //!< Stores editor settings for the profiler
        Configs::DebugSettings m_configDebug; //!< Stores editor settings for the profiler

        AZ::u32 SelectCsvPath();
    };
} // namespace FPSProfiler
