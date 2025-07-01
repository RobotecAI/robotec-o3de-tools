#pragma once

#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/Component.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>

namespace RobotecRecordingTools
{

    //! Cursor state control component.
    //! This component is used to show or hide cursor in game mode.
    class ScreenRecorderComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , public AZ::Render::FrameCaptureNotificationBus::Handler

    {
    public:
        AZ_COMPONENT(ScreenRecorderComponent, "{99edb3e8-9a6a-11ee-b9d1-0242ac120002}");

        ScreenRecorderComponent() = default;
        ~ScreenRecorderComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        void OnFrameCaptureFinished(AZ::Render::FrameCaptureResult result, const AZStd::string& info) override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;


    private:
        void CaptureScreen();

        AZStd::string m_imagePath = AZStd::string("screenshot");
        int m_frameCnt = 0;
        float m_interval = -1.0f; //s
        float m_captureFramerate = 30.0f; // 1/s
        bool m_enabled = false;
        bool m_captureInProgress = false;

        int m_timeSlowdownRate = 1;
        float m_tickScale = 1.0f; // [-]
        int m_ticksSinceLastCapture = 0;
        float m_timeSinceLastCapture = 0.0f;
        float m_initialTickScale = 1.0f; // [-]

    };
} // namespace RobotecRecordingTools
