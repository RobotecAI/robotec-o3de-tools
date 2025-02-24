
#pragma once

#include <Tools/FPSProfilerData.h>

#include <cfloat>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <FPSProfiler/FPSProfilerBus.h>

namespace FPSProfiler
{
    class FPSProfilerSystemComponent
        : public AZ::Component
        , protected FPSProfilerRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(FPSProfilerSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        FPSProfilerSystemComponent();
        FPSProfilerSystemComponent(const FPSProfilerData& m_Configuration);
        ~FPSProfilerSystemComponent();

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
        AZStd::vector<float> m_fpsSamples;
        AZStd::vector<AZStd::string> m_logEntries;

        float m_minFPS = FLT_MAX; // Tracking the lowest FPS value - set to max for difference
        float m_maxFPS = 0.0f; // Tracking the highest FPS value - set to min for diff
        float m_totalFrameTime = 0.0f;
        int m_frameCount = 0;

        FPSProfilerData m_ProfilerData;

        void WriteDataToFile();
    };

} // namespace FPSProfiler
