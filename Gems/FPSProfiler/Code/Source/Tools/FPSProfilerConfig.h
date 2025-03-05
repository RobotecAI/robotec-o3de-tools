#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace FPSProfiler
{
    enum MovingAverageType
    {
        Simple,
        Exponential,
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

    struct FPSProfilerConfigStats
    {
        bool m_SaveFpsData = true;
        bool m_SaveGpuData = true;
        bool m_SaveCpuData = true;
    };

    struct FPSProfilerConfigPrecision
    {
        float m_NearZeroPrecision = 0.01f;
        MovingAverageType m_avgFpsType = MovingAverageType::Exponential;
        bool m_AverageMedianFilter = true;
    };

    struct FPSProfilerConfigDebug
    {
        bool m_ShowFps = true;
    };

} // namespace FPSProfiler
