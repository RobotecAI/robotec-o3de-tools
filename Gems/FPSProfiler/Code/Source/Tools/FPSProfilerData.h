#pragma once

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace FPSProfiler
{
    struct FPSProfilerData
    {
        AZ_TYPE_INFO(FPSProfilerData, FPSProfilerDataTypeId);
        static void Reflect(AZ::ReflectContext* context);

        AZ::IO::Path m_OutputFilename = "@user@/fps_log.csv";
        bool m_SaveWithTimestamp = true;
        bool m_AutoSave = true;
        int m_AutoSaveOccurrences = 100;
        float m_NearZeroPrecision = 0.01f;
        bool m_SaveFpsData = true;
        bool m_SaveGpuData = true;
        bool m_SaveCpuData = true;
        bool m_ShowFps = true;
    };
} // namespace FPSProfiler
