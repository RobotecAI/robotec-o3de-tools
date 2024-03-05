#include "ScreenRecorderComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Time/TimeSystem.h>

namespace RobotecRecordingTools
{
    void ScreenRecorderComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ScreenRecorderComponent, AZ::Component>()
                ->Version(2)
                ->Field("Enabled", &ScreenRecorderComponent::m_enabled)
                ->Field("Output image path", &ScreenRecorderComponent::m_imagePath)
                ->Field("Capture framerate", &ScreenRecorderComponent::m_captureFramerate)
                ->Field("Tick scale", &ScreenRecorderComponent::m_tickScale);


            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<ScreenRecorderComponent>("Capture Screenshots", "Capture Screenshots")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "ScreenRecorderComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecRecordingTools")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ScreenRecorderComponent::m_enabled,
                        "Enable screen capturing",
                        "Enable screen capturing")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ScreenRecorderComponent::m_imagePath,
                        "Output image path",
                        "Path and file name (without extension). Relative path will be resolved in reference to \"{project_path}/Cache/{linux|windows|...}\". WARNING: \"~\" is not recognized as user's home.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ScreenRecorderComponent::m_captureFramerate,
                        "Capture framerate",
                        "Limit of capture framerate. This framerate coresponds simulation time, which can be scaled using \"Tick scale\" parameter. If this framerate if lower or equal zero, frames will be captured without limit.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ScreenRecorderComponent::m_tickScale,
                        "Tick scale",
                        "Simulation time scaling coefficient. 1.0 means no scaling.")
                    ;
            }
        }
    }

    void ScreenRecorderComponent::CaptureScreen()
    {

        AZStd::string imagePath = AZStd::string::format("%s%08i.png", m_imagePath.c_str(), m_frameCnt);
        AZ::Render::FrameCaptureOutcome captureOutcome;
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(captureOutcome, &AZ::Render::FrameCaptureRequestBus::Events::CaptureScreenshot, imagePath);

        AZ_Error("ScreenRecorderComponent", captureOutcome.IsSuccess(),
            "Frame capture initialization failed. %s", captureOutcome.GetError().m_errorMessage.c_str());

        if (captureOutcome.IsSuccess())
        {
            m_captureInProgress = true;
            AZ::Render::FrameCaptureNotificationBus::Handler::BusConnect(captureOutcome.GetValue());
        }
        m_frameCnt++;
    }

    void ScreenRecorderComponent::Activate()
    {
        if(m_enabled)
        {
            AZ::TickBus::Handler::BusConnect();

            if (m_tickScale > 0.0f)
            {
                if (auto* timeSystem = AZ::Interface<AZ::ITime>::Get())
                {
                        m_initialTickScale = timeSystem->GetSimulationTickScale();
                        timeSystem->SetSimulationTickScale(m_tickScale);
                }
            }
    
            m_interval = (m_captureFramerate > 0.0f) ? 1.0 / m_captureFramerate : -1.0f;
        }
    }

    void ScreenRecorderComponent::Deactivate()
    {
        if(m_enabled)
        {
            AZ::TickBus::Handler::BusDisconnect();

            if (m_tickScale > 0.0f)
            {
                if (auto* timeSystem = AZ::Interface<AZ::ITime>::Get())
                {
                        timeSystem->SetSimulationTickScale(m_initialTickScale);
                }
            }
        }
    }

    void ScreenRecorderComponent::OnFrameCaptureFinished([[maybe_unused]] AZ::Render::FrameCaptureResult result, [[maybe_unused]] const AZStd::string& info)
    {
        m_captureInProgress = false;
        AZ_Error("ScreenRecorderComponent", (result == AZ::Render::FrameCaptureResult::Success),
            "Frame capture failed. Info: %s", info.c_str());        
        AZ::Render::FrameCaptureNotificationBus::Handler::BusDisconnect();
    }

    void ScreenRecorderComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if(m_enabled)
        {
            m_ticksSinceLastCapture++;
            m_timeSinceLastCapture += deltaTime;
        
            if (m_timeSinceLastCapture > m_interval)
            {
                CaptureScreen();

                AZ_TracePrintf("ScreenRecorderComponent", " -- Tick cnt: %i   Frame time %fs", m_ticksSinceLastCapture, m_timeSinceLastCapture);
                m_ticksSinceLastCapture = 0;
                m_timeSinceLastCapture = 0.0f;
            }
        }
    }

} // namespace RobotecRecordingTools
