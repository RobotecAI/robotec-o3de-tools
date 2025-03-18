#include "FPSProfilerConfig.h"

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>

namespace FPSProfiler::Configs
{
    void FileSaveSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            if (serializeContext->FindClassData(azrtti_typeid<FileSaveSettings>())) // Prevent duplicate registration
            {
                return;
            }

            serializeContext->Class<FileSaveSettings>()
                ->Version(0)
                ->Field("m_OutputFilename", &FileSaveSettings::m_OutputFilename)
                ->Field("m_AutoSave", &FileSaveSettings::m_AutoSave)
                ->Field("m_AutoSaveAtFrame", &FileSaveSettings::m_AutoSaveAtFrame)
                ->Field("m_SaveWithTimestamp", &FileSaveSettings::m_SaveWithTimestamp);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<FileSaveSettings>("File Save Settings", "Settings for managing how FPS data is saved to a file.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Configure file-saving options for recorded FPS data.")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

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
                        "When enabled, system will auto save after specified frame occurrence. Recommended for optimization and long "
                        "recordings.")
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
                            const FileSaveSettings* data = static_cast<const FileSaveSettings*>(instance);
                            return data && data->m_AutoSave ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
                        })

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_SaveWithTimestamp,
                        "Timestamp",
                        "When enabled, the system will save files with a timestamp postfix of the current date, hour, minutes, and "
                        "seconds. This allows you to save automatically without manual input each time.");
            }
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            if (behaviorContext->m_classes.contains("FileSaveSettings"))
            {
                return;
            }

            behaviorContext->Class<FileSaveSettings>("FileSaveSettings")
                ->Attribute(AZ::Script::Attributes::Category, "FPSProfiler")
                ->Constructor<>()
                ->Property("outputFilename", BehaviorValueProperty(&FileSaveSettings::m_OutputFilename))
                ->Property("autoSave", BehaviorValueProperty(&FileSaveSettings::m_AutoSave))
                ->Property("autoSaveAtFrame", BehaviorValueProperty(&FileSaveSettings::m_AutoSaveAtFrame))
                ->Property("saveWithTimestamp", BehaviorValueProperty(&FileSaveSettings::m_SaveWithTimestamp));
        }
    }

    void RecordSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            if (serializeContext->FindClassData(azrtti_typeid<RecordSettings>())) // Prevent duplicate registration
            {
                return;
            }

            serializeContext->Class<RecordSettings>()
                ->Version(0)
                ->Field("m_recordType", &RecordSettings::m_recordType)
                ->Field("m_framesToSkip", &RecordSettings::m_framesToSkip)
                ->Field("m_framesToRecord", &RecordSettings::m_framesToRecord)
                ->Field("m_RecordStats", &RecordSettings::m_RecordStats);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<RecordSettings>("Recording Settings", "Options for configuring how FPS data is recorded.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Control the behavior of FPS data recording.")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &RecordSettings::m_recordType, "Record Type", "Specifies the type of record.")
                    ->EnumAttribute(RecordType::GameStart, "Game Start")
                    ->EnumAttribute(RecordType::FramePick, "Frame Pick")
                    ->EnumAttribute(RecordType::Await, "Await")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RecordSettings::m_framesToSkip,
                        "Frames To Skip",
                        "Number of frames to skip before starting recording.")
                    ->Attribute(
                        AZ::Edit::Attributes::Visibility,
                        [](const void* instance)
                        {
                            const RecordSettings* data = static_cast<const RecordSettings*>(instance);
                            return data && data->m_recordType == RecordType::FramePick ? AZ::Edit::PropertyVisibility::Show
                                                                                       : AZ::Edit::PropertyVisibility::Hide;
                        })
                    ->Attribute(AZ::Edit::Attributes::Min, 0)
                    ->Attribute(AZ::Edit::Attributes::Step, 1)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RecordSettings::m_framesToRecord,
                        "Frames To Record",
                        "Number of frames to capture. If set to 0.0f it will be skipped.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0)
                    ->Attribute(AZ::Edit::Attributes::Step, 100)

                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &RecordSettings::m_RecordStats,
                        "Record Stats",
                        "Specifies the type of stats that will be saved to file.")
                    ->EnumAttribute(RecordStatistics::None, "None")
                    ->EnumAttribute(RecordStatistics::FPS, "FPS")
                    ->EnumAttribute(RecordStatistics::CPU, "CPU")
                    ->EnumAttribute(RecordStatistics::GPU, "GPU")
                    ->EnumAttribute(RecordStatistics::MemoryUsage, "MemoryUsage")
                    ->EnumAttribute(RecordStatistics::All, "All")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree);
            }
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            if (behaviorContext->m_classes.contains("RecordSettings"))
            {
                return;
            }

            behaviorContext->Class<RecordSettings>("RecordSettings")
                ->Attribute(AZ::Script::Attributes::Category, "FPSProfiler")
                ->Constructor<>()
                ->Property("recordType", BehaviorValueProperty(&RecordSettings::m_recordType))
                ->Property("framesToSkip", BehaviorValueProperty(&RecordSettings::m_framesToSkip))
                ->Property("framesToRecord", BehaviorValueProperty(&RecordSettings::m_framesToRecord))
                ->Property("recordStats", BehaviorValueProperty(&RecordSettings::m_RecordStats));
        }
    }

    void PrecisionSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            if (serializeContext->FindClassData(azrtti_typeid<PrecisionSettings>())) // Prevent duplicate registration
            {
                return;
            }

            serializeContext->Class<PrecisionSettings>()
                ->Version(0)
                ->Field("m_NearZeroPrecision", &PrecisionSettings::m_NearZeroPrecision)
                ->Field("m_avgFpsType", &PrecisionSettings::m_avgFpsType)
                ->Field("m_smoothingFactor", &PrecisionSettings::m_smoothingFactor)
                ->Field("m_keepHistory", &PrecisionSettings::m_keepHistory);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PrecisionSettings>("Precision Settings", "Defines the precision level of the FPS Profiler measurements.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Adjust how precisely the FPS Profiler records data.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrecisionSettings::m_NearZeroPrecision,
                        "Near Zero Precision",
                        "Threshold for near-zero values")

                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &PrecisionSettings::m_avgFpsType,
                        "Moving Average Type",
                        "Select the type of moving average to use")
                    ->EnumAttribute(MovingAverageType::Simple, "Simple")
                    ->EnumAttribute(MovingAverageType::Exponential, "Exponential")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrecisionSettings::m_smoothingFactor,
                        "Alpha Smoothing Factor",
                        "Alpha Smoothing Factor for Exponential Average Calculation.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0)
                    ->Attribute(AZ::Edit::Attributes::Step, 0.1)
                    ->Attribute(
                        AZ::Edit::Attributes::Visibility,
                        [](const void* instance)
                        {
                            const PrecisionSettings* data = static_cast<const PrecisionSettings*>(instance);
                            return data && data->m_avgFpsType == Exponential ? AZ::Edit::PropertyVisibility::Show
                                                                             : AZ::Edit::PropertyVisibility::Hide;
                        })

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PrecisionSettings::m_keepHistory,
                        "Keep History",
                        "Enabled saves entire history for better avg fps smoothing, otherwise history is cleared per auto save if "
                        "enabled.");
            }
        }

        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            if (behaviorContext->m_classes.contains("PrecisionSettings"))
            {
                return;
            }

            behaviorContext->Class<PrecisionSettings>("PrecisionSettings")
                ->Attribute(AZ::Script::Attributes::Category, "FPSProfiler")
                ->Constructor<>()
                ->Property("NearZeroPrecision", BehaviorValueProperty(&PrecisionSettings::m_NearZeroPrecision))
                ->Property("AvgFpsType", BehaviorValueProperty(&PrecisionSettings::m_avgFpsType))
                ->Property("SmoothingFactor", BehaviorValueProperty(&PrecisionSettings::m_smoothingFactor))
                ->Property("KeepHistory", BehaviorValueProperty(&PrecisionSettings::m_keepHistory));
        }
    }

    void DebugSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            if (serializeContext->FindClassData(azrtti_typeid<DebugSettings>())) // Prevent duplicate registration
            {
                return;
            }

            serializeContext->Class<DebugSettings>()
                ->Version(0)
                ->Field("PrintDebugInfo", &DebugSettings::m_PrintDebugInfo)
                ->Field("ShowFps", &DebugSettings::m_ShowFps)
                ->Field("Color", &DebugSettings::m_Color);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<DebugSettings>("Debug Settings", "Configuration options for debugging the FPS Profiler.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Customize debugging settings for the FPS Profiler.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::CheckBox,
                        &DebugSettings::m_PrintDebugInfo,
                        "Print Debug Info",
                        "Enable or disable debug information printing.")

                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &DebugSettings::m_ShowFps, "Show FPS", "Toggle FPS display on screen.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Color, &DebugSettings::m_Color, "Debug Color", "Set the debug information display color.")
                    ->Attribute(
                        AZ::Edit::Attributes::Visibility,
                        [](const void* instance)
                        {
                            const DebugSettings* data = static_cast<const DebugSettings*>(instance);
                            return data && data->m_ShowFps ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
                        });
            }
        }

        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            if (behaviorContext->m_classes.contains("DebugSettings"))
            {
                return;
            }

            behaviorContext->Class<DebugSettings>("DebugSettings")
                ->Attribute(AZ::Script::Attributes::Category, "FPSProfiler")
                ->Constructor<>()
                ->Property("PrintDebugInfo", BehaviorValueProperty(&DebugSettings::m_PrintDebugInfo))
                ->Property("ShowFps", BehaviorValueProperty(&DebugSettings::m_ShowFps))
                ->Property("Color", BehaviorValueProperty(&DebugSettings::m_Color));
        }
    }
} // namespace FPSProfiler::Configs