#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/Math/Color.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace FPSProfiler::Configs
{
    enum MovingAverageType
    {
        Simple = 0,
        Exponential = 1,
    };
    AZ_DEFINE_ENUM_RELATIONAL_OPERATORS(MovingAverageType);

    enum RecordType : uint8_t
    {
        GameStart = 0,
        FramePick = 1,
        Await = 2,
    };
    AZ_DEFINE_ENUM_RELATIONAL_OPERATORS(RecordType);

    enum RecordStatistics : uint8_t
    {
        None = 0,
        FPS = 1 << 0,
        CPU = 1 << 1,
        GPU = 1 << 2,
        All = FPS | CPU | GPU,
        MemoryUsage = CPU | GPU,
    };
    AZ_DEFINE_ENUM_BITWISE_OPERATORS(RecordStatistics);
    AZ_DEFINE_ENUM_RELATIONAL_OPERATORS(RecordStatistics);

    struct FileSaveSettings
    {
        AZ_TYPE_INFO(FileSaveSettings, FPSProfilerConfigFileTypeId);
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_OutputFilename = "@user@/fps_log.csv";
        bool m_AutoSave = true;
        int m_AutoSaveAtFrame = 100;
        bool m_SaveWithTimestamp = true;
    };

    struct RecordSettings
    {
        AZ_TYPE_INFO(RecordSettings, FPSProfilerConfigRecordTypeId);
        static void Reflect(AZ::ReflectContext* context);

        RecordType m_recordType = RecordType::GameStart;
        int m_framesToSkip = 0; // Available only for FramePick
        int m_framesToRecord = 0;
        RecordStatistics m_RecordStats = RecordStatistics::All;
    };

    struct PrecisionSettings
    {
        AZ_TYPE_INFO(PrecisionSettings, FPSProfilerConfigPrecisionTypeId);
        static void Reflect(AZ::ReflectContext* context);

        float m_NearZeroPrecision = 0.01f;
        MovingAverageType m_avgFpsType = MovingAverageType::Simple;
        float m_smoothingFactor = 2.0f;
        bool m_useAvgMedianFilter = true;
    };

    struct DebugSettings
    {
        AZ_TYPE_INFO(DebugSettings, FPSProfilerConfigDebugTypeId);
        static void Reflect(AZ::ReflectContext* context);

        bool m_PrintDebugInfo = true;
        bool m_ShowFps = true;
        AZ::Color m_Color = AZ::Colors::DarkRed;
    };

} // namespace FPSProfiler::Configs
