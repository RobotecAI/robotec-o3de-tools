#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/Math/Color.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace FPSProfiler::Configs
{
    enum MovingAverageType : bool
    {
        Simple = true,
        Exponential = false,
    };

    enum RecordType
    {
        GameStart = 0,
        FramePick = 1,
        Await = 2,
    };

    enum RecordStatistics : uint8_t
    {
        None = 0,
        FPS  = 1 << 0,
        CPU  = 1 << 1,
        GPU  = 1 << 2,
        All  = FPS | CPU | GPU,
        Memory = CPU | GPU,
    };

    struct FileSaveSettings
    {
        AZ_TYPE_INFO(FileSaveSettings, FPSProfilerConfigFileTypeId);
        static void Reflect(AZ::ReflectContext* context);

        AZ::IO::Path m_OutputFilename = "@user@/fps_log.csv";
        bool m_AutoSave = true;
        int m_AutoSaveAtFrame = 100;
        bool m_SaveWithTimestamp = true;
    };

    struct RecordSettings
    {
        AZ_TYPE_INFO(RecordSettings, FPSProfilerConfigRecordTypeId);
        static void Reflect(AZ::ReflectContext* context);

        RecordType m_recordType = RecordType::GameStart;
        float m_framesToSkip = 0.0f;    // Available only for FramePick
        float m_framesToRecord = 0.0f;
        RecordStatistics m_RecordStats = RecordStatistics::All;
    };

    struct PrecisionSettings
    {
        AZ_TYPE_INFO(PrecisionSettings, FPSProfilerConfigPrecisionTypeId);
        static void Reflect(AZ::ReflectContext* context);

        float m_NearZeroPrecision = 0.01f;
        MovingAverageType m_avgFpsType = MovingAverageType::Exponential;
        bool m_useAvgMedianFilter = true;
    };

    struct DebugSettings
    {
        AZ_TYPE_INFO(DebugSettings, FPSProfilerConfigDebugTypeId);
        static void Reflect(AZ::ReflectContext* context);

        bool m_PrintDebugInfo = true;
        bool m_ShowFps = true;
        AZ::Color m_Color = AZ::Colors::Blue;
    };

} // namespace FPSProfiler::Configs
