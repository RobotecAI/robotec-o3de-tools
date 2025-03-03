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

    FPSProfilerSystemComponent::FPSProfilerSystemComponent(FPSProfilerConfig m_configuration, bool m_profileOnGameStart)
        : m_configuration(AZStd::move(m_configuration))
        , m_isProfiling(m_profileOnGameStart)
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
        if (!IsPathValid(m_configuration.m_OutputFilename))
        {
            m_configuration.m_OutputFilename = "@user@/fps_log.csv";
            AZ_Warning("FPSProfiler", false, "Invalid output file path. Using default: %s", m_configuration.m_OutputFilename.c_str());
        }

        FPSProfilerRequestBus::Handler::BusConnect(); // connect first to broadcast notifications
        ResetProfilingData();
        AZ::TickBus::Handler::BusConnect(); // connect last, after setup
        AZ_Printf("FPS Profiler", "Activating FPSProfiler");

        if (IsAnySaveOptionEnabled())
        {
            CreateLogFile();
        }

        // Reserve at least twice as needed occurrences, since close and save operation may happen at the tick frame saves.
        // Since log entries are cleared when occurrence update happens, it's good to reserve known size.
        if (m_configuration.m_AutoSave)
        {
            m_fpsSamples.reserve(m_configuration.m_AutoSaveAtFrame);
            m_logEntries.reserve(m_configuration.m_AutoSaveAtFrame * 2);
        }
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
        // Safety exit when profiling is disabled.
        if (!m_isProfiling)
        {
            return;
        }

        // Calculate data for Profiler Bus
        CalculateFpsData(deltaTime);

        if (m_configuration.m_ShowFps)
        {
            ShowFps();
        }

        // If none save option enabled - exit
        if (!IsAnySaveOptionEnabled())
        {
            return;
        }

        AZStd::string logEntry = AZStd::string::format(
            "%d,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            m_configuration.m_SaveFpsData ? m_frameCount : -1,
            m_configuration.m_SaveFpsData ? deltaTime : -1.0f,
            m_configuration.m_SaveFpsData ? m_currentFps : -1.0f,
            m_configuration.m_SaveFpsData ? m_minFps : -1.0f,
            m_configuration.m_SaveFpsData ? m_maxFps : -1.0f,
            m_configuration.m_SaveFpsData ? m_avgFps : -1.0f,
            m_configuration.m_SaveCpuData ? BytesToMB(GetCpuMemoryUsed()) : -1.0f,
            m_configuration.m_SaveGpuData ? BytesToMB(GetGpuMemoryUsed()) : -1.0f);
        m_logEntries.push_back(logEntry);

        // Save after every m_AutoSaveOccurrences frames to not overflow buffer, only when m_AutoSave enabled.
        if (m_configuration.m_AutoSave && m_frameCount % m_configuration.m_AutoSaveAtFrame == 0)
        {
            WriteDataToFile();
        }
    }

    int FPSProfilerSystemComponent::GetTickOrder()
    {
        return AZ::TICK_GAME;
    }

    void FPSProfilerSystemComponent::StartProfiling()
    {
        if (m_isProfiling)
        {
            return;
        }

        m_isProfiling = true;
        ResetProfilingData();
        CreateLogFile();

        if (!AZ::TickBus::Handler::BusIsConnected()) // Connect TickBus only if not already connected
        {
            AZ::TickBus::Handler::BusConnect();
        }
        AZ_Printf("FPS Profiler", "Profiling started.");
    }

    void FPSProfilerSystemComponent::StopProfiling()
    {
        if (!m_isProfiling)
        {
            return;
        }

        m_isProfiling = false;

        if (AZ::TickBus::Handler::BusIsConnected()) // Only disconnect if actually connected
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
        SaveLogToFile();
        AZ_Printf("FPS Profiler", "Profiling stopped.");
    }

    void FPSProfilerSystemComponent::ResetProfilingData()
    {
        m_minFps = AZ::Constants::FloatMax;
        m_maxFps = 0.0f;
        m_avgFps = 0.0f;
        m_currentFps = 0.0f;
        m_totalFrameTime = 0.0f;
        m_frameCount = 0;
        m_fpsSamples.clear();
        m_logEntries.clear();
    }

    bool FPSProfilerSystemComponent::IsProfiling() const
    {
        return m_isProfiling;
    }

    bool FPSProfilerSystemComponent::IsAnySaveOptionEnabled() const
    {
        return m_configuration.m_SaveFpsData || m_configuration.m_SaveCpuData || m_configuration.m_SaveGpuData;
    }

    void FPSProfilerSystemComponent::ChangeSavePath(const AZ::IO::Path& newSavePath)
    {
        if (!IsPathValid(newSavePath))
        {
            return;
        }

        m_configuration.m_OutputFilename = newSavePath;
        AZ_Warning("FPS Profiler", false, "Path changed.");
    }

    void FPSProfilerSystemComponent::SafeChangeSavePath(const AZ::IO::Path& newSavePath)
    {
        // If profiling is enabled, save current opened file and stop profiling.
        StopProfiling();
        ChangeSavePath(newSavePath);
    }

    float FPSProfilerSystemComponent::GetMinFps() const
    {
        return m_minFps;
    }

    float FPSProfilerSystemComponent::GetMaxFps() const
    {
        return m_maxFps;
    }

    float FPSProfilerSystemComponent::GetAvgFps() const
    {
        return m_avgFps;
    }

    float FPSProfilerSystemComponent::GetCurrentFps() const
    {
        return m_currentFps;
    }

    size_t FPSProfilerSystemComponent::GetCpuMemoryUsed() const
    {
        size_t usedBytes = 0;
        size_t reservedBytes = 0;

        // Get stats for the system allocator
        AZ::AllocatorManager::Instance().GetAllocatorStats(usedBytes, reservedBytes, nullptr);

        // Return the used bytes (allocated memory)
        return usedBytes;
    }

    size_t FPSProfilerSystemComponent::GetGpuMemoryUsed() const
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

    void FPSProfilerSystemComponent::SaveLogToFile()
    {
        WriteDataToFile();
    }

    void FPSProfilerSystemComponent::SaveLogToFileWithNewPath(const AZStd::string& newSavePath, bool useSafeChangePath)
    {
        if (useSafeChangePath)
        {
            SafeChangeSavePath(newSavePath);
        }
        else
        {
            ChangeSavePath(newSavePath);
        }

        WriteDataToFile();
    }

    void FPSProfilerSystemComponent::ShowFpsOnScreen(bool enable)
    {
        m_configuration.m_ShowFps = enable;
    }

    void FPSProfilerSystemComponent::CalculateFpsData(const float& deltaTime)
    {
        m_currentFps = deltaTime > 0 ? (1.0f / deltaTime) : 0.0f;
        m_fpsSamples.push_back(m_currentFps);

        m_totalFrameTime += deltaTime;
        m_frameCount++;

        // Using m_NearZeroPrecision, since m_currentFPS cannot be equal to 0 if delta time is valid.
        if (m_currentFps >= m_configuration.m_NearZeroPrecision)
        {
            m_minFps = AZStd::min(m_minFps, m_currentFps);
        }
        m_maxFps = AZStd::max(m_maxFps, m_currentFps);

        m_avgFps = !m_fpsSamples.empty()
            ? (AZStd::accumulate(m_fpsSamples.begin(), m_fpsSamples.end(), 0.0f) / static_cast<float>(m_fpsSamples.size()))
            : 0.0f;
    }

    void FPSProfilerSystemComponent::CreateLogFile()
    {
        if (!IsPathValid(m_configuration.m_OutputFilename))
        {
            m_configuration.m_OutputFilename = "@user@/fps_log.csv";
            AZ_Warning("FPSProfiler", false, "Invalid output file path. Using default: %s", m_configuration.m_OutputFilename.c_str());
        }

        // Apply Timestamp
        if (m_configuration.m_SaveWithTimestamp)
        {
            // Get current system time
            auto now = AZStd::chrono::system_clock::now();
            std::time_t now_time_t = AZStd::chrono::system_clock::to_time_t(now);

            // Convert to local time structure
            std::tm timeInfo{};
            localtime_r(&now_time_t, &timeInfo);

            // Format the timestamp as YYYYMMDD_HHMM
            char timestamp[16]; // Buffer for formatted time
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M", &timeInfo);

            m_configuration.m_OutputFilename.ReplaceFilename(
                (m_configuration.m_OutputFilename.Stem().String() + "_" + timestamp + m_configuration.m_OutputFilename.Extension().String())
                    .data());
        }

        // Validate if file can be created
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance();
        if (!fileIO->Exists(m_configuration.m_OutputFilename.c_str()))
        {
            AZ_Warning("FPSProfiler", false, "File does not exist, trying to create it...");

            AZ::IO::HandleType fileHandle;
            if (fileIO->Open(
                    m_configuration.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeWrite | AZ::IO::OpenMode::ModeCreatePath, fileHandle) ==
                AZ::IO::ResultCode::Success)
            {
                fileIO->Close(fileHandle);
                AZ_Printf("FPSProfiler", "Log file successfully created: %s", m_configuration.m_OutputFilename.c_str());
            }
            else
            {
                // Restore default path on fail
                m_configuration.m_OutputFilename = "@user@/fps_log.csv";
                AZ_Warning("FPSProfiler", false, "Failed to create file. Using default path: %s", m_configuration.m_OutputFilename.c_str());
            }
        }

        // Write profiling headers to file
        AZ::IO::FileIOStream file(m_configuration.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeWrite | AZ::IO::OpenMode::ModeCreatePath);
        AZStd::string csvHeader = "Frame,FrameTime,CurrentFPS,MinFPS,MaxFPS,AvgFPS,CpuMemoryUsed,GpuMemoryUsed\n";
        file.Write(csvHeader.size(), csvHeader.c_str());
        file.Close();

        // Notify - File Created
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnFileCreated, m_configuration.m_OutputFilename.c_str());
    }

    void FPSProfilerSystemComponent::WriteDataToFile()
    {
        if (m_logEntries.empty())
        {
            return;
        }

        if (!IsAnySaveOptionEnabled())
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

    float FPSProfilerSystemComponent::BytesToMB(size_t bytes)
    {
        return static_cast<float>(bytes) / (1024.0f * 1024.0f);
    }

    bool FPSProfilerSystemComponent::IsPathValid(const AZ::IO::Path& path)
    {
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance();

        if (path.empty() || !path.HasFilename() || !path.HasExtension() || !fileIO || !fileIO->ResolvePath(path.c_str()))
        {
            const char* reason = path.empty() ? "Path cannot be empty."
                : !path.HasFilename()         ? "Path must have a file at the end."
                : !path.HasExtension()        ? "Path must have a *.csv extension."
                : !fileIO                     ? "Could not get a FileIO object. Try again."
                                              : "Path is not registered or recognizable by O3DE FileIO System.";

            AZ_Warning("FPSProfiler::ChangeSavePath", false, "%s", reason);
            return false;
        }

        return true;
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
