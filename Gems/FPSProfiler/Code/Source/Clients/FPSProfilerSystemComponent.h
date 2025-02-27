#pragma once

#include <FPSProfiler/FPSProfilerBus.h>
#include <Tools/FPSProfilerData.h>

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
        explicit FPSProfilerSystemComponent(FPSProfilerData m_configuration);
        ~FPSProfilerSystemComponent() override;

    protected:
        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

    private:
        // Profiler Data - Editor Settings
        FPSProfilerData m_configuration;

        float m_minFps = 0.0; // Tracking the lowest FPS value
        float m_maxFps = 0.0f; // Tracking the highest FPS value
        float m_avgFps = 0.0f; // Mean Value of accumulated current FPS
        float m_currentFps = 0.0f; // Actual FPS in current frame
        float m_totalFrameTime = 0.0f; // Time it took to enter frame
        int m_frameCount = 0; // Numeric value of actual frame
        AZStd::vector<float> m_fpsSamples; // Vector of collected current FPSs. Cleared once @ref m_configuration.m_AutoSave enabled.
        AZStd::vector<AZStd::string> m_logEntries; // Vector of collected log entries. Cleared after @ref
                                                   // m_configuration.m_AutoSaveOccurrences, when @ref m_configuration.m_AutoSave enabled.

        // File operations
        void CreateLogFile();
        void WriteDataToFile();

        // Helpers
        void CalculateFpsData(const float& deltaTime);
        static float BytesToMB(size_t bytes);

        // Memory Access
        static size_t GetCpuMemoryUsed();
        static size_t GetGpuMemoryUsed();

        // Debug display
        void ShowFps() const;
    };
} // namespace FPSProfiler
