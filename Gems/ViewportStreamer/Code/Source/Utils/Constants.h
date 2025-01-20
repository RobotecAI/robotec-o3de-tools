#pragma once
#include <AzCore/base.h>

namespace ViewportStreamer::Constants
{
    constexpr char ViewportStreamerFrequencyRegistryKey[] = "/ViewportStreamer/StreamFrequency";
    constexpr char ViewportStreamerFrameNameRegistryKey[] = "/ViewportStreamer/FrameName";

    constexpr char ViewportStreamerFrameName[] = "viewportstreamer_frame";
    constexpr const AZ::u64 ViewportStreamerFrequency = 30;
} // namespace ViewportStreamer::Constants
