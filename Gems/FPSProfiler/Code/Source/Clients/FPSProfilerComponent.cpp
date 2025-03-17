#include "FPSProfilerComponent.h"

#include <Atom/RHI/Device.h>
#include <Atom/RHI/MemoryStatisticsBuilder.h>
#include <Atom/RHI/RHISystemInterface.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/numeric.h>

namespace FPSProfiler
{
    void FPSProfilerComponent::Reflect(AZ::ReflectContext* context)
    {
        Configs::FileSaveSettings::Reflect(context);
        Configs::RecordSettings::Reflect(context);
        Configs::PrecisionSettings::Reflect(context);
        Configs::DebugSettings::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerComponent, AZ::Component>()
                ->Version(0)
                ->Field("m_configFile", &FPSProfilerComponent::m_configFile)
                ->Field("m_configRecord", &FPSProfilerComponent::m_configRecord)
                ->Field("m_configPrecision", &FPSProfilerComponent::m_configPrecision)
                ->Field("m_configDebug", &FPSProfilerComponent::m_configDebug);
        }
    }

    void FPSProfilerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("FPSProfilerService"));
    }

    void FPSProfilerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("FPSProfilerService"));
    }

    FPSProfilerComponent::FPSProfilerComponent()
    {
        if (!FPSProfilerInterface::Get())
        {
            FPSProfilerInterface::Register(this);
        }
    }

    FPSProfilerComponent::FPSProfilerComponent(
        const Configs::FileSaveSettings& configF,
        const Configs::RecordSettings& configS,
        const Configs::PrecisionSettings& configP,
        const Configs::DebugSettings& configD)
        : m_configFile(AZStd::move(configF))
        , m_configRecord(AZStd::move(configS))
        , m_configPrecision(AZStd::move(configP))
        , m_configDebug(AZStd::move(configD))
    {
        if (!FPSProfilerInterface::Get())
        {
            FPSProfilerInterface::Register(this);
        }
    }

    FPSProfilerComponent::~FPSProfilerComponent()
    {
        if (FPSProfilerInterface::Get() == this)
        {
            FPSProfilerInterface::Unregister(this);
        }
    }

    void FPSProfilerComponent::Activate()
    {
        if (!IsPathValid(m_configFile.m_OutputFilename))
        {
            m_configFile.m_OutputFilename = "@user@/fps_log.csv";
            AZ_Warning(
                "FPSProfiler",
                !m_configDebug.m_PrintDebugInfo,
                "Invalid output file path. Using default: %s",
                m_configFile.m_OutputFilename.c_str());
        }

        // Reserve log entries buffer size based on known auto save per frame
        m_configFile.m_AutoSave ? m_logBuffer.reserve(MAX_LOG_BUFFER_LINE_SIZE * m_configFile.m_AutoSaveAtFrame * 2)
                                : m_logBuffer.reserve(MAX_LOG_BUFFER_SIZE);

        FPSProfilerRequestBus::Handler::BusConnect(); // connect first to broadcast notifications
        AZ::TickBus::Handler::BusConnect(); // connect last, after setup

        if (m_configRecord.m_recordType != Configs::RecordType::Await)
        {
            AZ::TickBus::QueueFunction(
                [this]()
                {
                    StartProfiling();
                });
        }

        AZ_Printf("FPS Profiler", "FPSProfiler activated.");
    }

    void FPSProfilerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();

        if (!m_configFile.m_AutoSave || m_configRecord.m_framesToRecord == 0 || m_logBuffer.size() < m_configFile.m_AutoSaveAtFrame)
        {
            WriteDataToFile();
        }

        // Notify - File Saved
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnFileSaved, m_configFile.m_OutputFilename.c_str());
        FPSProfilerRequestBus::Handler::BusDisconnect();
    }

    void FPSProfilerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_isProfiling)
        {
            return;
        }

        // Update FPS data
        CalculateFpsData(deltaTime);

        if (m_configDebug.m_ShowFps)
        {
            ShowFps();
        }

        if (m_configRecord.m_recordType == Configs::RecordType::FramePick && m_frameCount <= m_configRecord.m_framesToSkip)
        {
            // Wait for selected frame
            return;
        }

        if (m_configRecord.m_framesToRecord != 0 && m_configRecord.m_framesToRecord == m_recordedFrameCount++)
        {
            StopProfiling();
        }

        if (!IsAnySaveOptionEnabled())
        {
            return;
        }

        // Initialize log entry buffer
        char logEntry[MAX_LOG_BUFFER_LINE_SIZE];
        int logEntryLength = 0;

        // Initialize statistics data
        float usedCpu = -1.0f, reservedCpu = -1.0f;
        float usedGpu = -1.0f, reservedGpu = -1.0f;
        const char* logEntryFormat = "-1,-1.0,-1.0,-1.0,-1.0,-1.0,%.2f,%.2f,%.2f,%.2f\n";

        if (m_configRecord.m_RecordStats & Configs::RecordStatistics::CPU)
        {
            auto [cpuUsed, cpuReserved] = GetCpuMemoryUsed();
            usedCpu = BytesToMB(cpuUsed);
            reservedCpu = BytesToMB(cpuReserved);
        }

        if (m_configRecord.m_RecordStats & Configs::RecordStatistics::GPU)
        {
            auto [gpuUsed, gpuReserved] = GetGpuMemoryUsed();
            usedGpu = BytesToMB(gpuUsed);
            reservedGpu = BytesToMB(gpuReserved);
        }

        if (m_configRecord.m_RecordStats & Configs::RecordStatistics::FPS)
        {
            logEntryFormat = "%d,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n";
        }

        // Add log entry
        logEntryLength = azsnprintf(
            logEntry,
            MAX_LOG_BUFFER_LINE_SIZE,
            logEntryFormat,
            m_frameCount,
            deltaTime,
            m_currentFps,
            m_minFps,
            m_maxFps,
            m_avgFps,
            usedCpu,
            reservedCpu,
            usedGpu,
            reservedGpu);
        m_logBuffer.insert(m_logBuffer.end(), logEntry, logEntry + logEntryLength);

        // Auto save
        if (m_configFile.m_AutoSave && (m_frameCount % m_configFile.m_AutoSaveAtFrame == 0))
        {
            WriteDataToFile();
        }
    }

    int FPSProfilerComponent::GetTickOrder()
    {
        return AZ::TICK_GAME;
    }

    void FPSProfilerComponent::StartProfiling()
    {
        if (m_isProfiling)
        {
            AZ_Warning("FPS Profiler", !m_configDebug.m_PrintDebugInfo, "Profiler already activated.");
            return;
        }

        m_isProfiling = true;
        ResetProfilingData();
        CreateLogFile();

        // Connect TickBus only if not already connected
        if (!AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusConnect();
        }

        // Notify - Profile Started
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnProfileStart, m_configFile);
        AZ_Printf("FPS Profiler", "Profiling started.");
    }

    void FPSProfilerComponent::StopProfiling()
    {
        if (!m_isProfiling)
        {
            AZ_Warning("FPS Profiler", !m_configDebug.m_PrintDebugInfo, "Profiler already stopped.");
            return;
        }

        m_isProfiling = false;

        // Disconnect TickBus only if actually connected
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
        SaveLogToFile();

        // Notify - Profile Stopped
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnProfileStop, m_configFile);
        AZ_Printf("FPS Profiler", "Profiling stopped.");
    }

    void FPSProfilerComponent::ResetProfilingData()
    {
        m_minFps = AZ::Constants::FloatMax; // Start at max to ensure the first valid FPS sets the minimum correctly in AZStd::min
        m_maxFps = 0.0f;
        m_avgFps = 0.0f;
        m_currentFps = 0.0f;
        m_totalFrameTime = 0.0f;
        m_frameCount = 0;
        m_recordedFrameCount = 0;
        m_fpsSamples.clear();
        m_logBuffer.clear();

        // Notify - Profile Reset
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnProfileReset, m_configFile);
        AZ_Printf("FPS Profiler", "Profiling data reseted.");
    }

    bool FPSProfilerComponent::IsProfiling() const
    {
        return m_isProfiling;
    }

    bool FPSProfilerComponent::IsAnySaveOptionEnabled() const
    {
        return m_configRecord.m_RecordStats != Configs::RecordStatistics::None;
    }

    void FPSProfilerComponent::ChangeSavePath(const AZStd::string& newSavePath)
    {
        if (!IsPathValid(newSavePath))
        {
            return;
        }

        m_configFile.m_OutputFilename = newSavePath;
        AZ_Warning("FPS Profiler", !m_configDebug.m_PrintDebugInfo && !m_isProfiling, "Path changed during activated profiling.");
    }

    void FPSProfilerComponent::SafeChangeSavePath(const AZStd::string& newSavePath)
    {
        // If profiling is enabled, save current opened file and stop profiling.
        StopProfiling();
        ChangeSavePath(newSavePath);
    }

    float FPSProfilerComponent::GetMinFps() const
    {
        return m_minFps;
    }

    float FPSProfilerComponent::GetMaxFps() const
    {
        return m_maxFps;
    }

    float FPSProfilerComponent::GetAvgFps() const
    {
        return m_avgFps;
    }

    float FPSProfilerComponent::GetCurrentFps() const
    {
        return m_currentFps;
    }

    AZStd::pair<AZStd::size_t, AZStd::size_t> FPSProfilerComponent::GetCpuMemoryUsed() const
    {
        AZStd::size_t usedBytes = 0;
        AZStd::size_t reservedBytes = 0;

        AZ::AllocatorManager::Instance().GetAllocatorStats(usedBytes, reservedBytes);
        return { usedBytes, reservedBytes };
    }

    AZStd::pair<AZStd::size_t, AZStd::size_t> FPSProfilerComponent::GetGpuMemoryUsed() const
    {
        if (AZ::RHI::RHISystemInterface* rhiSystem = AZ::RHI::RHISystemInterface::Get())
        {
            if (AZ::RHI::Device* device = rhiSystem->GetDevice())
            {
                AZ::RHI::MemoryStatistics memoryStats;
                device->CompileMemoryStatistics(memoryStats, AZ::RHI::MemoryStatisticsReportFlags::Basic);

                return { memoryStats.m_heaps.front().m_memoryUsage.m_totalResidentInBytes,
                         memoryStats.m_heaps.front().m_memoryUsage.m_budgetInBytes };
            }
        }

        return { 0, 0 };
    }

    void FPSProfilerComponent::SaveLogToFile()
    {
        WriteDataToFile();
    }

    void FPSProfilerComponent::SaveLogToFileWithNewPath(const AZStd::string& newSavePath, bool useSafeChangePath)
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

    void FPSProfilerComponent::ShowFpsOnScreen(bool enable)
    {
        m_configDebug.m_ShowFps = enable;
    }

    void FPSProfilerComponent::CalculateFpsData(const float& deltaTime)
    {
        m_currentFps = deltaTime > 0 ? (1.0f / deltaTime) : 0.0f;
        m_totalFrameTime += deltaTime;
        m_frameCount++;

        // Latest fps history for avg fps calculation
        m_fpsSamples.push_back(m_currentFps);
        if (m_fpsSamples.size() > m_configFile.m_AutoSaveAtFrame)
        {
            m_fpsSamples.pop_front();
        }

        if (m_configPrecision.m_avgFpsType == Configs::MovingAverageType::Simple)
        {
            m_avgFps = AZStd::accumulate(m_fpsSamples.begin(), m_fpsSamples.end(), 0.0f) / static_cast<float>(m_fpsSamples.size());
        }
        else
        {
            const float alpha = m_configPrecision.m_smoothingFactor / (m_configFile.m_AutoSaveAtFrame + 1); // Smoothing factor
            // Compute EMA
            if (m_fpsSamples.size() == 1)
            {
                // Initialize EMA with the first FPS value
                m_avgFps = m_currentFps;
            }
            else
            {
                // Apply EMA formula
                m_avgFps = (alpha * m_currentFps) + ((1.0f - alpha) * m_avgFps);
            }
        }

        // Using m_NearZeroPrecision, since m_currentFPS cannot be equal to 0 if delta time is valid.
        if (m_currentFps >= m_configPrecision.m_NearZeroPrecision)
        {
            m_minFps = AZStd::min(m_minFps, m_currentFps);
        }

        m_maxFps = AZStd::max(m_maxFps, m_currentFps);
    }

    void FPSProfilerComponent::CreateLogFile()
    {
        if (!IsAnySaveOptionEnabled())
        {
            AZ_Warning("FPSProfiler", !m_configDebug.m_PrintDebugInfo, "None save option selected. Skipping file creation.");
            return;
        }

        if (!IsPathValid(m_configFile.m_OutputFilename))
        {
            m_configFile.m_OutputFilename = "@user@/fps_log.csv";
            AZ_Warning(
                "FPSProfiler",
                !m_configDebug.m_PrintDebugInfo,
                "Invalid output file path. Using default: %s",
                m_configFile.m_OutputFilename.c_str());
        }

        // Apply Timestamp
        if (m_configFile.m_SaveWithTimestamp)
        {
            auto now = AZStd::chrono::system_clock::now();
            std::time_t now_time_t = AZStd::chrono::system_clock::to_time_t(now);

            std::tm timeInfo{};
            localtime_r(&now_time_t, &timeInfo);

            // Format the timestamp as YYYYMMDD_HHMMSS
            char timestamp[20];
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeInfo);

            static_cast<AZ::IO::Path>(m_configFile.m_OutputFilename)
                .ReplaceFilename((static_cast<AZ::IO::Path>(m_configFile.m_OutputFilename).Stem().String() + "_" + timestamp +
                                  static_cast<AZ::IO::Path>(m_configFile.m_OutputFilename).Extension().String())
                                     .data());
        }

        // Write profiling headers to file
        AZ::IO::FileIOStream file(m_configFile.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeWrite | AZ::IO::OpenMode::ModeCreatePath);
        AZStd::string csvHeader =
            "Frame,FrameTime,CurrentFPS,MinFPS,MaxFPS,AvgFPS,CpuMemoryUsed,CpuMemoryReserved,GpuMemoryUsed,GpuMemoryReserved\n";
        file.Write(csvHeader.size(), csvHeader.c_str());
        file.Close();

        // Notify - File Created
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnFileCreated, m_configFile.m_OutputFilename.c_str());
    }

    void FPSProfilerComponent::WriteDataToFile()
    {
        if (m_logBuffer.empty())
        {
            return;
        }

        if (!IsAnySaveOptionEnabled())
        {
            return;
        }

        AZ::IO::HandleType file;
        if (AZ::IO::FileIOBase::GetInstance()->Open(m_configFile.m_OutputFilename.c_str(), AZ::IO::OpenMode::ModeAppend, file))
        {
            AZ::IO::FileIOBase::GetInstance()->Write(file, m_logBuffer.data(), m_logBuffer.size());
            AZ::IO::FileIOBase::GetInstance()->Close(file);
        }
        m_logBuffer.clear();

        // Notify - File Update
        FPSProfilerNotificationBus::Broadcast(&FPSProfilerNotifications::OnFileUpdate, m_configFile.m_OutputFilename.c_str());
    }

    float FPSProfilerComponent::BytesToMB(AZStd::size_t bytes)
    {
        return static_cast<float>(bytes) / (1024.0f * 1024.0f);
    }

    bool FPSProfilerComponent::IsPathValid(const AZStd::string& path) const
    {
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance();

        AZ::IO::Path tempPath(path.c_str());

        if (tempPath.empty() || !tempPath.HasFilename() || !tempPath.HasExtension() || !fileIO || !fileIO->ResolvePath(tempPath))
        {
            const char* reason = tempPath.empty() ? "Path cannot be empty."
                : !tempPath.HasFilename()         ? "Path must have a file at the end."
                : !tempPath.HasExtension()        ? "Path must have a *.csv extension."
                : !fileIO                         ? "Could not get a FileIO object. Try again."
                                                  : "Path is not registered or recognizable by O3DE FileIO System.";

            AZ_Warning("FPSProfiler::IsPathValid", !m_configDebug.m_PrintDebugInfo, "%s", reason);
            return false;
        }

        return true;
    }

    void FPSProfilerComponent::ShowFps() const
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
