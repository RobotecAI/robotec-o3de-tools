#include "FPSProfilerConfig.h"

#include <AzCore/Serialization/EditContext.h>

namespace FPSProfiler
{
    void FPSProfilerConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerConfig>()
                ->Version(0)
                ->Field("m_OutputFilename", &FPSProfilerConfig::m_OutputFilename)
                ->Field("m_SaveWithTimestamp", &FPSProfilerConfig::m_SaveWithTimestamp)
                ->Field("m_AutoSave", &FPSProfilerConfig::m_AutoSave)
                ->Field("m_AutoSaveOccurrences", &FPSProfilerConfig::m_AutoSaveOccurrences)
                ->Field("m_NearZeroPrecision", &FPSProfilerConfig::m_NearZeroPrecision)
                ->Field("m_SaveFPSData", &FPSProfilerConfig::m_SaveFpsData)
                ->Field("m_SaveCPUData", &FPSProfilerConfig::m_SaveCpuData)
                ->Field("m_SaveGPUData", &FPSProfilerConfig::m_SaveGpuData)
                ->Field("m_ShowFPS", &FPSProfilerConfig::m_ShowFps);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<FPSProfilerConfig>("FPS Profiler Configuration", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_OutputFilename,
                        "Csv Save Path",
                        "Select a path where *.csv will be saved.")
                    ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.csv")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_AutoSave,
                        "Auto Save",
                        "When enabled, system will auto save after specified frame occurrance.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_AutoSaveOccurrences,
                        "Auto Save At Frame",
                        "Specify after how many frames system will auto save log.")
                    ->Attribute(AZ::Edit::Attributes::Min, 1)
                    ->Attribute(
                        AZ::Edit::Attributes::Visibility,
                        [](const void* instance)
                        {
                            const FPSProfilerConfig* data = reinterpret_cast<const FPSProfilerConfig*>(instance);
                            return data && data->m_AutoSave ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
                        })

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_SaveWithTimestamp,
                        "Timestamp",
                        "When enabled, system will save files with timestamp postfix of current date and hour.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Precision Settings")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_NearZeroPrecision,
                        "Near Zero Precision",
                        "Specify near Zero precision, that will be used for system.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 0.1f)
                    ->Attribute(AZ::Edit::Attributes::Step, 0.00001f)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Data Settings")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_SaveFpsData,
                        "Save FPS Data",
                        "When enabled, system will collect FPS data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_SaveGpuData,
                        "Save GPU Data",
                        "When enabled, system will collect GPU usage data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_SaveCpuData,
                        "Save CPU Data",
                        "When enabled, system will collect CPU usage data into csv.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Debug Settings")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfig::m_ShowFps,
                        "Show FPS",
                        "When enabled, system will show FPS counter in top-left corner.");
            }
        }
    }
} // namespace FPSProfiler