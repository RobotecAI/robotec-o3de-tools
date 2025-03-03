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
        explicit FPSProfilerSystemComponent(FPSProfilerConfig m_configuration, bool m_profileOnGameStart);
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
        [[nodiscard]] bool IsProfiling() const override;
        [[nodiscard]] bool IsAnySaveOptionEnabled() const override;
        void ChangeSavePath(const AZ::IO::Path& newSavePath) override;
        void SafeChangeSavePath(const AZ::IO::Path& newSavePath) override;
        [[nodiscard]] float GetMinFps() const override;
        [[nodiscard]] float GetMaxFps() const override;
        [[nodiscard]] float GetAvgFps() const override;
        [[nodiscard]] float GetCurrentFps() const override;
        [[nodiscard]] size_t GetCpuMemoryUsed() const override;
        [[nodiscard]] size_t GetGpuMemoryUsed() const override;
        void SaveLogToFile() override;
        void SaveLogToFileWithNewPath(const AZ::IO::Path& newSavePath, bool useSafeChangePath) override;
        void ShowFpsOnScreen(bool enable) override;

    private:
        // Profiler Configuration - Editor Settings
        FPSProfilerConfig m_configuration;

        // Profiler Data
        bool m_isProfiling = false;
        float m_minFps = 0.0f; // Tracking the lowest FPS value
        float m_maxFps = 0.0f; // Tracking the highest FPS value
        float m_avgFps = 0.0f; // Mean Value of accumulated current FPS
        float m_currentFps = 0.0f; // Actual FPS in current frame
        float m_totalFrameTime = 0.0f; // Time it took to enter frame
        int m_frameCount = 0; // Numeric value of actual frame
        AZStd::deque<float> m_fpsSamples; // Vector of collected current FPSs. Cleared once @ref m_configuration.m_AutoSave enabled.
        AZStd::vector<AZStd::string> m_logEntries; // Vector of collected log entries. Cleared after @ref
                                                   // m_configuration.m_AutoSaveOccurrences, when @ref m_configuration.m_AutoSave enabled.

        // File operations
        void CreateLogFile();
        void WriteDataToFile();

        // Helpers
        void CalculateFpsData(const float& deltaTime);
        static float BytesToMB(size_t bytes);
        static bool IsPathValid(const AZ::IO::Path& path);

        // Debug display
        void ShowFps() const;
    };
} // namespace FPSProfiler
