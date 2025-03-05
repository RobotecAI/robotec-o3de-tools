#include "FPSProfilerConfig.h"

#include <AzCore/Serialization/EditContext.h>

namespace FPSProfiler
{
    void FPSProfilerConfigFile::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerConfigFile>()
                ->Version(0)
                ->Field("m_OutputFilename", &FPSProfilerConfigFile::m_OutputFilename)
                ->Field("m_AutoSave", &FPSProfilerConfigFile::m_AutoSave)
                ->Field("m_AutoSaveAtFrame", &FPSProfilerConfigFile::m_AutoSaveAtFrame)
                ->Field("m_SaveWithTimestamp", &FPSProfilerConfigFile::m_SaveWithTimestamp)
                ->Field("m_SaveFPSData", &FPSProfilerConfigFile::m_SaveFpsData)
                ->Field("m_SaveCPUData", &FPSProfilerConfigFile::m_SaveCpuData)
                ->Field("m_SaveGPUData", &FPSProfilerConfigFile::m_SaveGpuData)
                ->Field("m_NearZeroPrecision", &FPSProfilerConfigFile::m_NearZeroPrecision)
                ->Field("m_ShowFPS", &FPSProfilerConfigFile::m_ShowFps);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<FPSProfilerConfigFile>(
                        "FPS Profiler Configuration", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "File Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_OutputFilename,
                        "Csv Save Path",
                        "Select a path where *.csv will be saved.")
                    ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.csv")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_AutoSave,
                        "Auto Save",
                        "When enabled, system will auto save after specified frame occurrence.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_AutoSaveAtFrame,
                        "Auto Save At Frame",
                        "Specify after how many frames system will auto save log.")
                    ->Attribute(AZ::Edit::Attributes::Min, 1)
                    ->Attribute(
                        AZ::Edit::Attributes::Visibility,
                        [](const void* instance)
                        {
                            const FPSProfilerConfigFile* data = reinterpret_cast<const FPSProfilerConfigFile*>(instance);
                            return data && data->m_AutoSave ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
                        })

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_SaveWithTimestamp,
                        "Timestamp",
                        "When enabled, system will save files with timestamp postfix of current date and hour.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Statistics Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_SaveFpsData,
                        "Save FPS Data",
                        "When enabled, system will collect FPS data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_SaveGpuData,
                        "Save GPU Data",
                        "When enabled, system will collect GPU usage data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_SaveCpuData,
                        "Save CPU Data",
                        "When enabled, system will collect CPU usage data into csv.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Precision Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_NearZeroPrecision,
                        "Near Zero Precision",
                        "Specify near Zero precision, that will be used for system.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 0.1f)
                    ->Attribute(AZ::Edit::Attributes::Step, 0.00001f)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Debug Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerConfigFile::m_ShowFps,
                        "Show FPS",
                        "When enabled, system will show FPS counter in top-left corner.");
            }
        }
    }
} // namespace FPSProfiler