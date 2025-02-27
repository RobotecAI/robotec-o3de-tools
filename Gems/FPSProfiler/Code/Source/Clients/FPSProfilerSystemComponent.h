#pragma once

#include <FPSProfiler/FPSProfilerBus.h>
#include <Tools/FPSProfilerConfig.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

namespace FPSProfiler
{
    class FPSProfilerSystemComponent final
        : public AZ::Component
        , protected FPSProfilerRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(FPSProfilerSystemComponent, FPSProfilerSystemComponentTypeId, Component);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        FPSProfilerSystemComponent();
        explicit FPSProfilerSystemComponent(FPSProfilerConfig m_configuration);
        ~FPSProfilerSystemComponent() override;

    protected:
        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

        // FPSProfilerRequestBus::Handler implementation
        void StartProfiling() override;
        void StopProfiling() override;
        void ResetProfilingData() override;
        bool IsProfiling() const override;
        bool IsAnySaveOptionEnabled() const override;
        float GetMinFps() const override;
        float GetMaxFps() const override;
        float GetAvgFps() const override;
        float GetCurrentFps() const override;
        size_t GetCpuMemoryUsed() const override;
        size_t GetGpuMemoryUsed() const override;
        void SaveLogToFile() override;
        void ShowFpsOnScreen(bool enable) override;

    private:
        // Profiler Configuration - Editor Settings
        FPSProfilerConfig m_configuration;

        // Profiler Data
        bool m_isProfiling;
        float m_minFps; // Tracking the lowest FPS value
        float m_maxFps; // Tracking the highest FPS value
        float m_avgFps; // Mean Value of accumulated current FPS
        float m_currentFps; // Actual FPS in current frame
        float m_totalFrameTime; // Time it took to enter frame
        int m_frameCount; // Numeric value of actual frame
        AZStd::vector<float> m_fpsSamples; // Vector of collected current FPSs. Cleared once @ref m_configuration.m_AutoSave enabled.
        AZStd::vector<AZStd::string> m_logEntries; // Vector of collected log entries. Cleared after @ref
                                                   // m_configuration.m_AutoSaveOccurrences, when @ref m_configuration.m_AutoSave enabled.

        // File operations
        void CreateLogFile();
        void WriteDataToFile();

        // Helpers
        void CalculateFpsData(const float& deltaTime);
        static float BytesToMB(size_t bytes);

        // Debug display
        void ShowFps() const;
    };
} // namespace FPSProfiler
