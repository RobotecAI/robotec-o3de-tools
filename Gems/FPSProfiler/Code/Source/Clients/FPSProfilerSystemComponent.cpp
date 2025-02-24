
#include "FPSProfilerSystemComponent.h"

#include "Atom/RHI/RHISystemInterface.h"
#include "Atom/RPI.Public/RPISystemInterface.h"
#include "AzCore/IO/FileIO.h"

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace FPSProfiler
{
    AZ_COMPONENT_IMPL(FPSProfilerSystemComponent, "FPSProfilerSystemComponent",
        FPSProfilerSystemComponentTypeId);

    void FPSProfilerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerSystemComponent, AZ::Component>()
                ->Version(0);
        }
    }

    void FPSProfilerSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("FPSProfilerService"));
    }

    void FPSProfilerSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("FPSProfilerService"));
    }

    void FPSProfilerSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void FPSProfilerSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    FPSProfilerSystemComponent::FPSProfilerSystemComponent()
    {
        if (FPSProfilerInterface::Get() == nullptr)
        {
            FPSProfilerInterface::Register(this);
        }
    }

    FPSProfilerSystemComponent::FPSProfilerSystemComponent(const FPSProfilerData& m_Configuration)
        : m_ProfilerData(m_Configuration)
    {
    }

    FPSProfilerSystemComponent::~FPSProfilerSystemComponent()
    {
        if (FPSProfilerInterface::Get() == this)
        {
            FPSProfilerInterface::Unregister(this);
        }
    }

    void FPSProfilerSystemComponent::Init(q)
    {
    }

    void FPSProfilerSystemComponent::Activate()
    {
        FPSProfilerRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        if (m_ProfilerData.m_OutputFilename.empty())
        {
            AZ_Error("FPSProfiler", false, "The output filename must be provided or cannot be empty!");
            return;
        }

        AZ::IO::FileIOStream file(m_ProfilerData.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeWrite);
        AZStd::string csvHeader = "Frame,FrameTime,InstantFPS,MinFPS,MaxFPS,AvgFPS,GpuMemoryUsed\n";
        file.Write(csvHeader.size(), csvHeader.c_str());
        file.Close();

        AZ_Printf("FPS Profiler", "FPS Profiler Activated.");
    }

    void FPSProfilerSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        WriteDataToFile();

        FPSProfilerRequestBus::Handler::BusDisconnect();
    }

    void FPSProfilerSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        float fps = deltaTime > 0 ? (1.0f / deltaTime) : 0.0f;

        m_fpsSamples.push_back(fps);
        m_minFPS = AZStd::min(m_minFPS, fps);
        m_maxFPS = AZStd::max(m_maxFPS, fps);
        m_totalFrameTime += deltaTime;
        m_frameCount++;

        float avgFPS = (m_frameCount > 0) ? (m_frameCount / m_totalFrameTime) : 0.0f;

        float gpuMemoryUsed = 0.0f;
        if (AZ::RPI::RPISystemInterface* rpiSystem = AZ::RPI::RPISystemInterface::Get())
        {
            // gpuMemoryUsed = static_cast<float>(rpiSystem->GetCurrentCpuMemoryUsage());
            [[maybe_unused]]AZ::RHI::RHISystemInterface* rhiSystem = AZ::RHI::RHISystemInterface::Get();
            AZ::RHI::MemoryStatistics memoryStatistics;
        }

        AZStd::string logEntry = AZStd::string::format(
            "%d,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            m_frameCount, deltaTime, fps, m_minFPS, m_maxFPS, avgFPS, gpuMemoryUsed
        );
        m_logEntries.push_back(logEntry);

        if (m_frameCount % 100 == 0)
        {
            WriteDataToFile();
        }
    }

    int FPSProfilerSystemComponent::GetTickOrder()
    {
        return AZ::TICK_GAME;
    }

    void FPSProfilerSystemComponent::WriteDataToFile()
    {
        if (!m_logEntries.empty())
        {
            AZ::IO::FileIOStream file(m_ProfilerData.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeAppend | AZ::IO::OpenMode::ModeWrite);

            for (const auto& entry : m_logEntries)
            {
                file.Write(entry.size(), entry.c_str());
            }
            file.Close();
            m_logEntries.clear();
        }
    }
} // namespace FPSProfiler
