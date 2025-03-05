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
                ->Field("m_SaveWithTimestamp", &FileSaveSettings::m_SaveWithTimestamp);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<FileSaveSettings>("FPS Profiler Configuration", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
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
                            const FileSaveSettings* data = static_cast<const FileSaveSettings*>(instance);
                            return data && data->m_AutoSave ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
                        })

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FileSaveSettings::m_SaveWithTimestamp,
                        "Timestamp",
                        "When enabled, system will save files with timestamp postfix of current date and hour.");
            }
        }
    }

    void RecordSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RecordSettings>()
                ->Version(0)
                ->Field("m_recordType", &RecordSettings::m_recordType)
                ->Field("m_framesToSkip", &RecordSettings::m_framesToSkip)
                ->Field("m_framesToRecord", &RecordSettings::m_framesToRecord)
                ->Field("m_RecordStats", &RecordSettings::m_RecordStats);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<RecordSettings>("Record Settings", "Settings controlling the recording behavior.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)

                    // Reflect enum with ComboBox:
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &RecordSettings::m_recordType, "Record Type", "Specifies the type of record.")
                    // Provide the combo‐box choices:
                    ->EnumAttribute(static_cast<int>(RecordType::GameStart), "Game Start")
                    ->EnumAttribute(static_cast<int>(RecordType::FramePick), "Frame Pick")
                    ->EnumAttribute(static_cast<int>(RecordType::Await), "Await")
                    // Ensure the UI updates when the enum changes:
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)

                    // Conditionally show m_framesToSkip only if "Frame Pick" is selected:
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

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RecordSettings::m_recordType,
                        "Record Type",
                        "Specifies the type of desired record.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RecordSettings::m_framesToSkip,
                        "Frames To Skip",
                        "Number of frames to skip before starting recording.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Step, 1.0f)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RecordSettings::m_framesToRecord,
                        "Frames To Record",
                        "Number of frames to capture. If set to 0.0f it will be skipped.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Step, 100.0f)

                    // FPS Button
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Toggle Save FPS", "Toggle FPS recording")
                    ->Attribute(
                        AZ::Edit::Attributes::ButtonText,
                        [](void* instance) -> AZStd::string
                        {
                            auto* self = reinterpret_cast<RecordSettings*>(instance);
                            return (self->m_RecordStats & RecordStatistics::FPS) ? "Disable FPS" : "Enable FPS";
                        })
                    ->Attribute(
                        AZ::Edit::Attributes::ChangeNotify,
                        [](void* instance)
                        {
                            auto* self = reinterpret_cast<RecordSettings*>(instance);
                            self->m_RecordStats = static_cast<RecordStatistics>(self->m_RecordStats ^ RecordStatistics::FPS); // Toggle bit
                            return AZ::Edit::PropertyRefreshLevels::ValuesOnly;
                        })

                    // CPU Button
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Toggle Save CPU", "Toggle CPU recording")
                    ->Attribute(
                        AZ::Edit::Attributes::ButtonText,
                        [](void* instance) -> AZStd::string
                        {
                            auto* self = reinterpret_cast<RecordSettings*>(instance);
                            return (self->m_RecordStats & RecordStatistics::CPU) ? "Disable CPU" : "Enable CPU";
                        })
                    ->Attribute(
                        AZ::Edit::Attributes::ChangeNotify,
                        [](void* instance)
                        {
                            auto* self = reinterpret_cast<RecordSettings*>(instance);
                            self->m_RecordStats = static_cast<RecordStatistics>(self->m_RecordStats ^ RecordStatistics::CPU); // Toggle bit
                            return AZ::Edit::PropertyRefreshLevels::ValuesOnly;
                        })

                    // GPU Button
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Save Save GPU", "Toggle GPU recording")
                    ->Attribute(
                        AZ::Edit::Attributes::ButtonText,
                        [](void* instance) -> AZStd::string
                        {
                            auto* self = reinterpret_cast<RecordSettings*>(instance);
                            return (self->m_RecordStats & RecordStatistics::GPU) ? "Disable GPU" : "Enable GPU";
                        })
                    ->Attribute(
                        AZ::Edit::Attributes::ChangeNotify,
                        [](void* instance)
                        {
                            auto* self = reinterpret_cast<RecordSettings*>(instance);
                            self->m_RecordStats = static_cast<RecordStatistics>(self->m_RecordStats ^ RecordStatistics::GPU); // Toggle bit
                            return AZ::Edit::PropertyRefreshLevels::ValuesOnly;
                        });
            }
        }
    }

    void PrecisionSettings::Reflect(AZ::ReflectContext* context)
    {
    }

    void DebugSettings::Reflect(AZ::ReflectContext* context)
    {
    }
} // namespace FPSProfiler::Configs