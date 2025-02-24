#include "FPSProfilerData.h"

#include <AzCore/Serialization/EditContext.h>

namespace FPSProfiler
{
    void FPSProfilerData::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerData>()
                ->Version(0)
                ->Field("m_OutputFilename", &FPSProfilerData::m_OutputFilename)
                ->Field("m_SaveMultiple", &FPSProfilerData::m_SaveMultiple)
                ->Field("m_SaveFPSData", &FPSProfilerData::m_SaveFPSData)
                ->Field("m_SaveCPUData", &FPSProfilerData::m_SaveCPUData)
                ->Field("m_SaveGPUData", &FPSProfilerData::m_SaveGPUData)
                ->Field("m_ShowFPS", &FPSProfilerData::m_ShowFPS);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<FPSProfilerData>("FPS Profiler Configuration", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerData::m_OutputFilename, "Csv Save Path", "Select a path where *.csv will be saved.")
                        ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.csv")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerData::m_SaveMultiple, "Save Multiple",
                        "When enabled, system will save files without overwriting current file. Each file will have prefix *_n.csv postfix numeration.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Data Settings")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerData::m_SaveFPSData, "Save FPS Data",
                                "When enabled, system will collect FPS data into csv.")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerData::m_SaveGPUData, "Save GPU Data",
                            "When enabled, system will collect GPU usage data into csv.")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerData::m_SaveCPUData, "Save CPU Data",
                                "When enabled, system will collect CPU usage data into csv.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Debug Settings")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerData::m_ShowFPS, "Show FPS",
                            "When enabled, system will show FPS counter in top-left corner.");
            }
        }
    }
} // FPSProfiler