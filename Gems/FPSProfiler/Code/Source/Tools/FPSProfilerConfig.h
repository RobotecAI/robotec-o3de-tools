#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/Math/Color.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace FPSProfiler::Config
{
    enum MovingAverageType
    {
        Simple,
        Exponential,
    };

    enum RecordStats : u_int8_t
    {
        None = 1 << 0,
        FPS = 1 << 1,
        CPU = 1 << 2,
        GPU = 1 << 3,
    };

    struct FPSProfilerConfigFile
    {
        AZ_TYPE_INFO(FPSProfilerConfigFile, FPSProfilerConfigFileTypeId);
        static void Reflect(AZ::ReflectContext* context);

        AZ::IO::Path m_OutputFilename = "@user@/fps_log.csv";
        bool m_AutoSave = true;
        int m_AutoSaveAtFrame = 100;
        bool m_SaveWithTimestamp = true;
        bool m_SaveFpsData = true;
        bool m_SaveGpuData = true;
        bool m_SaveCpuData = true;
        float m_NearZeroPrecision = 0.01f;
        MovingAverageType m_avgFpsType = MovingAverageType::Exponential;
        bool m_AverageMedianFilter = true;
        bool m_ShowFps = true;
    };

    struct Stats
    {
        bool m_SaveFpsData = true;
        bool m_SaveGpuData = true;
        bool m_SaveCpuData = true;
    };

    struct Precision
    {
        float m_NearZeroPrecision = 0.01f;
        MovingAverageType m_avgFpsType = MovingAverageType::Exponential;
        bool m_AverageMedianFilter = true;
    };

    struct Debug
    {
        bool m_ShowFps = true;
        AZ::Color m_Color = AZ::Colors::Blue;
    };

} // namespace FPSProfiler::Config
