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
        bool m_SaveMultiple = true;
        bool m_SaveFPSData = true;
        bool m_SaveGPUData = true;
        bool m_SaveCPUData = true;
        bool m_ShowFPS = true;
    };
} // namespace FPSProfiler
