#include "FPSProfilerConfig.h"

#include <AzCore/Serialization/EditContext.h>

namespace FPSProfiler::Configs
{
    void FileSaveSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FileSaveSettings>()
                ->Version(0)
                ->Field("m_OutputFilename", &FileSaveSettings::m_OutputFilename)
                ->Field("m_AutoSave", &FileSaveSettings::m_AutoSave)
                ->Field("m_AutoSaveAtFrame", &FileSaveSettings::m_AutoSaveAtFrame)
                ->Field("m_SaveWithTimestamp", &FileSaveSettings::m_SaveWithTimestamp)
                ->Field("m_SaveFPSData", &FileSaveSettings::m_SaveFpsData)
                ->Field("m_SaveCPUData", &FileSaveSettings::m_SaveCpuData)
                ->Field("m_SaveGPUData", &FileSaveSettings::m_SaveGpuData)
                ->Field("m_NearZeroPrecision", &FileSaveSettings::m_NearZeroPrecision)
                ->Field("m_ShowFPS", &FileSaveSettings::m_ShowFps);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<FileSaveSettings>(
                        "FPS Profiler Configuration", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "File Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_OutputFilename,
                        "Csv Save Path",
                        "Select a path where *.csv will be saved.")
                    ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.csv")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_AutoSave,
                        "Auto Save",
                        "When enabled, system will auto save after specified frame occurrence.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_AutoSaveAtFrame,
                        "Auto Save At Frame",
                        "Specify after how many frames system will auto save log.")
                    ->Attribute(AZ::Edit::Attributes::Min, 1)
                    ->Attribute(
                        AZ::Edit::Attributes::Visibility,
                        [](const void* instance)
                        {
                            const FileSaveSettings* data = reinterpret_cast<const FileSaveSettings*>(instance);
                            return data && data->m_AutoSave ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
                        })

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_SaveWithTimestamp,
                        "Timestamp",
                        "When enabled, system will save files with timestamp postfix of current date and hour.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Statistics Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_SaveFpsData,
                        "Save FPS Data",
                        "When enabled, system will collect FPS data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_SaveGpuData,
                        "Save GPU Data",
                        "When enabled, system will collect GPU usage data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_SaveCpuData,
                        "Save CPU Data",
                        "When enabled, system will collect CPU usage data into csv.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Precision Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_NearZeroPrecision,
                        "Near Zero Precision",
                        "Specify near Zero precision, that will be used for system.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 0.1f)
                    ->Attribute(AZ::Edit::Attributes::Step, 0.00001f)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Debug Settings")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_ShowFps,
                        "Show FPS",
                        "When enabled, system will show FPS counter in top-left corner.");
            }
        }
    }
} // namespace FPSProfiler::Configs