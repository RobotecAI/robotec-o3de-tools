
#include "FPSProfilerEditorSystemComponent.h"

#include "AzQtComponents/Components/Widgets/FileDialog.h"
#include "UI/UICore/WidgetHelpers.h"

#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace FPSProfiler
{
    AZ_COMPONENT_IMPL(FPSProfilerEditorSystemComponent, "FPSProfilerEditorSystemComponent",
        FPSProfilerEditorSystemComponentTypeId, BaseSystemComponent);

    void FPSProfilerEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FPSProfilerEditorSystemComponent, FPSProfilerSystemComponent>()
                ->Version(0)
                ->Field("m_OutputFilename", &FPSProfilerEditorSystemComponent::m_OutputFilename)
                ->Field("m_SaveMultiple", &FPSProfilerEditorSystemComponent::m_SaveMultiple)
                ->Field("m_SaveFPSData", &FPSProfilerEditorSystemComponent::m_SaveFPSData)
                ->Field("m_SaveGPUData", &FPSProfilerEditorSystemComponent::m_SaveGPUData)
                ->Field("m_SaveCPUData", &FPSProfilerEditorSystemComponent::m_SaveCPUData)
                ->Field("m_ShowFPS", &FPSProfilerEditorSystemComponent::m_ShowFPS);


            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<FPSProfilerEditorSystemComponent>("FPS Profiler", "Tracks FPS, GPU and CPU performance and saves it into .csv")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Performance")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, false)

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_OutputFilename, "Csv Save Path", "Select a path where *.csv will be saved.")
                        ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.csv")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_SaveMultiple, "Save Multiple",
                        "When enabled, system will save files without overwriting current file. Each file will have prefix *_n.csv postfix numeration.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Data Settings")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_SaveFPSData, "Save FPS Data",
                                "When enabled, system will collect FPS data into csv.")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_SaveGPUData, "Save GPU Data",
                            "When enabled, system will collect GPU usage data into csv.")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_SaveCPUData, "Save CPU Data",
                                "When enabled, system will collect CPU usage data into csv.")

                    ->ClassElement(AZ::Edit::ClassElements::Group, "Debug Settings")

                    ->DataElement(AZ::Edit::UIHandlers::Default, &FPSProfilerEditorSystemComponent::m_ShowFPS, "Show FPS",
                            "When enabled, system will show FPS counter in top-left corner.")
                    ;
            }
        }
    }

    FPSProfilerEditorSystemComponent::FPSProfilerEditorSystemComponent() = default;

    FPSProfilerEditorSystemComponent::~FPSProfilerEditorSystemComponent() = default;

    void FPSProfilerEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("FPSProfilerEditorService"));
    }

    void FPSProfilerEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("FPSProfilerEditorService"));
    }

    void FPSProfilerEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void FPSProfilerEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void FPSProfilerEditorSystemComponent::Activate()
    {
        FPSProfilerSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void FPSProfilerEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        FPSProfilerSystemComponent::Deactivate();
    }
} // namespace FPSProfiler
