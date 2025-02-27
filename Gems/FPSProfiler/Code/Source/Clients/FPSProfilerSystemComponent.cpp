#include "FPSProfilerSystemComponent.h"

#include <Atom/RHI/Device.h>
#include <Atom/RHI/MemoryStatisticsBuilder.h>
#include <Atom/RHI/RHISystemInterface.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/numeric.h>

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
        if (m_configuration.m_OutputFilename.empty())
        {
            AZ_Error("FPSProfiler", false, "The output filename must be provided or cannot be empty!");
            return;
        }

        // Reserve at least twice as needed occurrences, since close and save operation may happen at the tick frame saves.
        // Since log entries are cleared when occurrence update happens, it's good to reserve known size.
        if (m_configuration.m_AutoSave)
        {
            m_fpsSamples.reserve(m_configuration.m_AutoSaveOccurrences * 2);
            m_logEntries.reserve(m_configuration.m_AutoSaveOccurrences * 2);
        }

        if (m_configuration.m_SaveFpsData)
        {
            // Cannot be 0 for std::min comparison
            m_minFps = AZ::Constants::FloatMax;
        }

        FPSProfilerRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
        CreateLogFile();
    }

    void FPSProfilerSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        WriteDataToFile();

        // Notify - File Saved
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnFileSaved, m_configuration.m_OutputFilename.c_str());

        FPSProfilerRequestBus::Handler::BusDisconnect();
    }

    void FPSProfilerSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_configuration.m_ShowFps)
        {
            ShowFps();
        }

        // Calculate data only if enabled, otherwise push default values to log entry.
        if (m_configuration.m_SaveFpsData)
        {
            CalculateFpsData(deltaTime);
        }

        AZStd::string logEntry = AZStd::string::format(
            "%d,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            m_frameCount,
            deltaTime,
            m_currentFps,
            m_minFps,
            m_maxFps,
            m_avgFps,
            m_configuration.m_SaveCPUData ? BytesToMB(GetCpuMemoryUsed()) : 0.0f,
            m_configuration.m_SaveGPUData ? BytesToMB(GetGpuMemoryUsed()) : 0.0f);
        m_logEntries.push_back(logEntry);

        // Save after every m_AutoSaveOccurrences frames to not overflow buffer, only when m_AutoSave enabled.
        if (m_configuration.m_AutoSave && m_frameCount % m_configuration.m_AutoSaveOccurrences == 0)
        {
            WriteDataToFile();
        }
    }

    int FPSProfilerSystemComponent::GetTickOrder()
    {
        return AZ::TICK_GAME;
    }

    void FPSProfilerSystemComponent::CalculateFpsData(const float& deltaTime)
    {
        m_currentFps = deltaTime > 0 ? (1.0f / deltaTime) : 0.0f;
        m_fpsSamples.push_back(m_currentFps);

        m_totalFrameTime += deltaTime;
        m_frameCount++;

        // Using m_NearZeroPrecision, since m_currentFPS cannot be equal to 0 if delta time is valid.
        if (m_currentFps > m_configuration.m_NearZeroPrecision)
        {
            m_minFps = AZStd::min(m_minFps, m_currentFps);
        }
        m_maxFps = AZStd::max(m_maxFps, m_currentFps);

        m_avgFps = !m_fpsSamples.empty() ? (AZStd::accumulate(m_fpsSamples.begin(), m_fpsSamples.end(), 0.0f) / m_fpsSamples.size()) : 0.0f;
    }

    void FPSProfilerSystemComponent::CreateLogFile()
    {
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance();
        bool fileExists = fileIO->Exists(m_configuration.m_OutputFilename.c_str());

        if (!fileExists)
        {
            m_configuration.m_OutputFilename = "@user@/fps_log.csv"; // Restore to default path
            AZ_Error(
                "FPSProfiler::CreateLogFile",
                false,
                "Specified file does not exist. Using default path: %s",
                m_configuration.m_OutputFilename.c_str());
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
        AZStd::string csvHeader = "Frame,FrameTime,CurrentFPS,MinFPS,MaxFPS,AvgFPS,CpuMemoryUsed,GpuMemoryUsed\n";
        file.Write(csvHeader.size(), csvHeader.c_str());
        file.Close();

        // Notify - File Created
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnFileCreated, m_configuration.m_OutputFilename.c_str());
    }

    void FPSProfilerSystemComponent::WriteDataToFile()
    {
        // Exit when nothing to save
        if (m_logEntries.empty())
        {
            return;
        }

        AZ::IO::FileIOStream file(m_configuration.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeAppend);

        for (const auto& entry : m_logEntries)
        {
            file.Write(entry.size(), entry.c_str());
        }
        file.Close();
        m_logEntries.clear();
        m_fpsSamples.clear();

        // Notify - File Update
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnFileUpdate, m_configuration.m_OutputFilename.c_str());
    }

    size_t FPSProfilerSystemComponent::GetCpuMemoryUsed()
    {
        size_t usedBytes = 0;
        size_t reservedBytes = 0;

        // Get stats for the system allocator
        AZ::AllocatorManager::Instance().GetAllocatorStats(usedBytes, reservedBytes, nullptr);

        // Return the used bytes (allocated memory)
        return usedBytes;
    }

    size_t FPSProfilerSystemComponent::GetGpuMemoryUsed()
    {
        if (AZ::RHI::RHISystemInterface* rhiSystem = AZ::RHI::RHISystemInterface::Get())
        {
            if (AZ::RHI::Device* device = rhiSystem->GetDevice())
            {
                AZ::RHI::MemoryStatistics memoryStats;
                device->CompileMemoryStatistics(memoryStats, AZ::RHI::MemoryStatisticsReportFlags::Detail);

                // Return the GPU memory used in bytes
                return memoryStats.m_heaps.front().m_memoryUsage.m_totalResidentInBytes;
            }
        }

        return 0;
    }

    float FPSProfilerSystemComponent::BytesToMB(size_t bytes)
    {
        return static_cast<float>(bytes) / (1024.0f * 1024.0f);
    }

    void FPSProfilerSystemComponent::ShowFps() const
    {
        AzFramework::DebugDisplayRequestBus::BusPtr debugDisplayBus;
        AzFramework::DebugDisplayRequestBus::Bind(debugDisplayBus, AzFramework::g_defaultSceneEntityDebugDisplayId);
        AzFramework::DebugDisplayRequests* debugDisplay = AzFramework::DebugDisplayRequestBus::FindFirstHandler(debugDisplayBus);

        if (!debugDisplay)
        {
            return;
        }

        debugDisplay->SetColor(AZ::Colors::Red);
        debugDisplay->SetAlpha(1.0f);

        AZStd::string debugText = AZStd::string::format("Profiler | FPS: %.2f", m_currentFps);
        debugDisplay->Draw2dTextLabel(10, 10, 1.0f, debugText.c_str(), true);
    }
} // namespace FPSProfiler
