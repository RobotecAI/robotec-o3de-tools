#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace FPSProfiler
{
    struct FPSProfilerConfig
    {
        AZ_TYPE_INFO(FPSProfilerConfig, FPSProfilerDataTypeId);
        static void Reflect(AZ::ReflectContext* context);

        AZ::IO::Path m_OutputFilename = "@user@/fps_log.csv";
        bool m_AutoSave = true;
        int m_AutoSaveOccurrences = 100;
        bool m_SaveWithTimestamp = true;
        float m_NearZeroPrecision = 0.01f;
        bool m_SaveFpsData = true;
        bool m_SaveGpuData = true;
        bool m_SaveCpuData = true;
        bool m_ShowFps = true;
    };
} // namespace FPSProfiler
