#pragma once

#include <Configurations/FPSProfilerConfig.h>
#include <FPSProfiler/FPSProfilerBus.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

namespace FPSProfiler
{
    class FPSProfilerComponent final
        : public AZ::Component
        , protected AZ::TickBus::Handler
        , protected FPSProfilerRequestBus::Handler
    {
    public:
        AZ_COMPONENT(FPSProfilerComponent, FPSProfilerComponentTypeId, Component);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        FPSProfilerComponent();
        explicit FPSProfilerComponent(
            const Configs::FileSaveSettings& configF,
            const Configs::RecordSettings& configS,
            const Configs::PrecisionSettings& configP,
            const Configs::DebugSettings& configD);
        ~FPSProfilerComponent() override;

        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

    protected:
        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

        // FPSProfilerRequestBus::Handler implementation
        void StartProfiling() override;
        void StopProfiling() override;
        void ResetProfilingData() override;
        [[nodiscard]] bool IsProfiling() const override;
        [[nodiscard]] bool IsAnySaveOptionEnabled() const override;
        void ChangeSavePath(const AZStd::string& newSavePath) override;
        void SafeChangeSavePath(const AZStd::string& newSavePath) override;
        [[nodiscard]] float GetMinFps() const override;
        [[nodiscard]] float GetMaxFps() const override;
        [[nodiscard]] float GetAvgFps() const override;
        [[nodiscard]] float GetCurrentFps() const override;
        [[nodiscard]] AZStd::pair<AZStd::size_t, AZStd::size_t> GetCpuMemoryUsed() const override;
        [[nodiscard]] AZStd::pair<AZStd::size_t, AZStd::size_t> GetGpuMemoryUsed() const override;
        void SaveLogToFile() override;
        void SaveLogToFileWithNewPath(const AZStd::string& newSavePath, bool useSafeChangePath) override;
        void ShowFpsOnScreen(bool enable) override;

    public:
        // Profiler Configurations
        Configs::FileSaveSettings m_configFile; //!< Stores editor settings for the profiler
        Configs::RecordSettings m_configRecord; //!< Stores editor settings for the profiler
        Configs::PrecisionSettings m_configPrecision; //!< Stores editor settings for the profiler
        Configs::DebugSettings m_configDebug; //!< Stores editor settings for the profiler

    private:
        // FPS Tracking Data
        bool m_isProfiling = false; //!< Flag to indicate if profiling is active
        float m_minFps = 0.0f; //!< Lowest FPS value recorded
        float m_maxFps = 0.0f; //!< Highest FPS value recorded
        float m_avgFps = 0.0f; //!< Mean value of collected FPS samples
        float m_currentFps = 0.0f; //!< FPS in the current frame
        float m_totalFrameTime = 0.0f; //!< Time taken for the current frame
        int m_frameCount = 0; //!< Total number of frames processed
        int m_recordedFrameCount = 0; //!< Total number of frames recorded. Used when @ref m_configRecord.m_framesToRecord != 0
        AZStd::deque<float> m_fpsSamples; //!< Stores recent FPS values for averaging

        // Log Buffer
        AZStd::vector<char> m_logBuffer; //!< Buffer for log entries, cleared periodically if auto save enabled.
        static constexpr AZStd::size_t MAX_LOG_BUFFER_SIZE = 1024 * 128; //!< Max log buffer size
        static constexpr AZStd::size_t MAX_LOG_BUFFER_LINE_SIZE = 128; //!< Max length per log line

        // File Operations
        void CreateLogFile();
        void WriteDataToFile();

        // Utility Functions
        void CalculateFpsData(const float& deltaTime);
        static float BytesToMB(AZStd::size_t bytes);
        [[nodiscard]] bool IsPathValid(const AZStd::string& path) const;

        // Debug Display
        void ShowFps() const;
    };
} // namespace FPSProfiler
