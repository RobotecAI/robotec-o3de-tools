#include "FPSProfilerData.h"

#include <AzCore/Serialization/EditContext.h>

namespace FPSProfiler
{
    void FPSProfilerData::Reflect(AZ::ReflectContext* context)
    {
        if (!context)
        {
            return;
        }

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerData>()
                ->Version(0)
                ->Field("m_OutputFilename", &FPSProfilerData::m_OutputFilename)
                ->Field("m_SaveWithTimestamp", &FPSProfilerData::m_SaveWithTimestamp)
                ->Field("m_AutoSave", &FPSProfilerData::m_AutoSave)
                ->Field("m_AutoSaveOccurrences", &FPSProfilerData::m_AutoSaveOccurrences)
                ->Field("m_NearZeroPrecision", &FPSProfilerData::m_NearZeroPrecision)
                ->Field("m_SaveFPSData", &FPSProfilerData::m_SaveFPSData)
                ->Field("m_SaveCPUData", &FPSProfilerData::m_SaveCPUData)
                ->Field("m_SaveGPUData", &FPSProfilerData::m_SaveGPUData)
                ->Field("m_ShowFPS", &FPSProfilerData::m_ShowFPS);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<FPSProfilerData>("FPS Profiler Configuration", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_OutputFilename,
                        "Csv Save Path",
                        "Select a path where *.csv will be saved.")
                    ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.csv")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_SaveWithTimestamp,
                        "Timestamp",
                        "When enabled, system will save files with timestamp postfix of current date and hour.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_AutoSave,
                        "Auto Save",
                        "When enabled, system will auto save after specified frame occurrance.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_AutoSaveOccurrences,
                        "Auto Save At Frame",
                        "Specify after how many frames system will auto save log.")
                    ->Attribute(AZ::Edit::Attributes::Min, 1)
                    ->Attribute(
                        AZ::Edit::Attributes::Visibility,
                        [](const void* instance)
                        {
                            const FPSProfilerData* data = reinterpret_cast<const FPSProfilerData*>(instance);
                            return data && data->m_AutoSave ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
                        })

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_NearZeroPrecision,
                        "Near Zero Precision",
                        "Specify near Zero precision, that will be used for system.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 0.1f)

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Data Settings")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_SaveFPSData,
                        "Save FPS Data",
                        "When enabled, system will collect FPS data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_SaveGPUData,
                        "Save GPU Data",
                        "When enabled, system will collect GPU usage data into csv.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_SaveCPUData,
                        "Save CPU Data",
                        "When enabled, system will collect CPU usage data into csv.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Debug Settings")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FPSProfilerData::m_ShowFPS,
                        "Show FPS",
                        "When enabled, system will show FPS counter in top-left corner.");
            }
        }
    }
} // namespace FPSProfiler