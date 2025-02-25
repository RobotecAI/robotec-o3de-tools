#include "FPSProfilerSystemComponent.h"

#include <AzQtComponents/Components/Widgets/FileDialog.h>
#include <AzToolsFramework/UI/UICore/WidgetHelpers.h>

#include <Atom/RHI/RHISystemInterface.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

namespace FPSProfiler
{
    void FPSProfilerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerSystemComponent, AZ::Component>()->Version(0)->Field(
                "m_Configuration", &FPSProfilerSystemComponent::m_configuration);
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

    FPSProfilerSystemComponent::FPSProfilerSystemComponent()
    {
        if (FPSProfilerInterface::Get() == nullptr)
        {
            FPSProfilerInterface::Register(this);
        }
    }

    FPSProfilerSystemComponent::FPSProfilerSystemComponent(FPSProfilerData m_configuration)
        : m_configuration(AZStd::move(m_configuration))
    {
        if (FPSProfilerInterface::Get() == nullptr)
        {
            FPSProfilerInterface::Register(this);
        }
    }

    FPSProfilerSystemComponent::~FPSProfilerSystemComponent()
    {
        if (FPSProfilerInterface::Get() == this)
        {
            FPSProfilerInterface::Unregister(this);
        }
    }

    void FPSProfilerSystemComponent::Activate()
    {
        FPSProfilerRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();

        if (m_configuration.m_OutputFilename.empty())
        {
            AZ_Error("FPSProfiler", false, "The output filename must be provided or cannot be empty!");
            return;
        }

        CreateLogFile();

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
        ShowFPS(fps);

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
            [[maybe_unused]] AZ::RHI::RHISystemInterface* rhiSystem = AZ::RHI::RHISystemInterface::Get();
            AZ::RHI::MemoryStatistics memoryStatistics;
        }

        AZStd::string logEntry = AZStd::string::format(
            "%d,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f\n", m_frameCount, deltaTime, fps, m_minFPS, m_maxFPS, avgFPS, gpuMemoryUsed);
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

    void FPSProfilerSystemComponent::CreateLogFile()
    {
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance();
        bool fileExists = fileIO->Exists(m_configuration.m_OutputFilename.c_str());

        if (!fileExists)
        {
            m_configuration.m_OutputFilename = "@user@/fps_log.csv"; // Restore to default path
        }

        if (m_configuration.m_SaveWithTimestamp)
        {
            // Get current system time
            auto now = AZStd::chrono::system_clock::now();
            std::time_t now_time_t = AZStd::chrono::system_clock::to_time_t(now);

            // Convert to local time structure
            std::tm timeInfo;
            localtime_r(&now_time_t, &timeInfo);

            // Format the timestamp as YYYYMMDD_HHMM
            char timestamp[16]; // Buffer for formatted time
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M", &timeInfo);

            m_configuration.m_OutputFilename.ReplaceFilename(
                (m_configuration.m_OutputFilename.Stem().String() + "_" + timestamp + m_configuration.m_OutputFilename.Extension().String())
                    .data());
        }

        AZ::IO::FileIOStream file(m_configuration.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeWrite | AZ::IO::OpenMode::ModeCreatePath);
        AZStd::string csvHeader = "Frame,FrameTime,InstantFPS,MinFPS,MaxFPS,AvgFPS,GpuMemoryUsed\n";
        file.Write(csvHeader.size(), csvHeader.c_str());
        file.Close();
    }

    void FPSProfilerSystemComponent::WriteDataToFile()
    {
        if (!m_logEntries.empty())
        {
            AZ::IO::FileIOStream file(m_configuration.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeAppend | AZ::IO::OpenMode::ModeWrite);

            for (const auto& entry : m_logEntries)
            {
                file.Write(entry.size(), entry.c_str());
            }
            file.Close();
            m_logEntries.clear();
        }
    }

    void FPSProfilerSystemComponent::ShowFPS(const float& fps) const
    {
        if (!m_configuration.m_ShowFPS)
        {
            return;
        }

        AzFramework::DebugDisplayRequestBus::BusPtr debugDisplayBus;
        AzFramework::DebugDisplayRequestBus::Bind(debugDisplayBus, AzFramework::g_defaultSceneEntityDebugDisplayId);
        AzFramework::DebugDisplayRequests* debugDisplay = AzFramework::DebugDisplayRequestBus::FindFirstHandler(debugDisplayBus);

        if (!debugDisplay)
        {
            return;
        }

        debugDisplay->SetColor(AZ::Colors::DarkRed);
        debugDisplay->SetAlpha(0.8f);

        AZStd::string debugText = AZStd::string::format("Profiler | FPS: %.2f", fps);
        debugDisplay->Draw2dTextLabel(10, 10, 1.0f, debugText.c_str(), true);
    }
} // namespace FPSProfiler
