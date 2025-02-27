#pragma once

#include <FPSProfiler/FPSProfilerBus.h>
#include <Tools/FPSProfilerData.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

namespace FPSProfiler
{
    class FPSProfilerSystemComponent
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
        explicit FPSProfilerSystemComponent(FPSProfilerData m_configuration);
        ~FPSProfilerSystemComponent() override;

    protected:
        ////////////////////////////////////////////////////////////////////////
        // FPSProfilerRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;
        ////////////////////////////////////////////////////////////////////////
    private:
        AZStd::vector<float> m_fpsSamples; // Vector of collected current FPSs. Cleared once @ref m_configuration.m_AutoSave enabled.
        AZStd::vector<AZStd::string> m_logEntries; // Vector of collected log entries. Cleared after @ref
                                                   // m_configuration.m_AutoSaveOccurrences, when @ref m_configuration.m_AutoSave enabled.

        float m_minFPS = AZ::Constants::FloatMax; // Tracking the lowest FPS value - set to max for difference
        float m_maxFPS = 0.0f; // Tracking the highest FPS value - set to min for diff
        float m_avgFPS = 0.0f; // Mean Value of accumulated current FPS
        float m_currentFPS = 0.0f; // Actual FPS in current frame
        float m_totalFrameTime = 0.0f;
        int m_frameCount = 0;
        void CalculateFpsData(const float& deltaTime);

        // Profiler Data - Editor Settings
        FPSProfilerData m_configuration;

        // File operations
        void CreateLogFile();
        void WriteDataToFile();

        // Memory Access
        static size_t GetCpuMemoryUsed();
        static size_t GetGpuMemoryUsed();
        static float BytesToMB(size_t bytes);

        // Debug display
        void ShowFps() const;
    };

} // namespace FPSProfiler
