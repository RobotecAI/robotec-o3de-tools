#include "SplineSubscriberConfig.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace SplineTools
{
    SplineSubscriberConfiguration::SplineSubscriberConfiguration()
    {
        m_topic.m_type = "geometry_msgs/msg/PoseStamped";
        m_topic.m_topic = "spline";
    }

    void SplineSubscriberConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SplineSubscriberConfiguration>()
                ->Version(0)
                ->Field("m_topicName", &SplineSubscriberConfiguration::m_topic)
                ->Field("m_allowWGS84", &SplineSubscriberConfiguration::m_allowWGS84)
                ->Field("m_resetOnActivation", &SplineSubscriberConfiguration::m_resetOnActivation)
                ->Field("m_startOffsetTag", &SplineSubscriberConfiguration::m_startOffsetTag);
            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<SplineSubscriberConfiguration>(
                        "SplineSubscriberConfiguration", "Configuration for the SplineSubscriber component")
                    ->ClassElement(AZ::Edit::ClassElements::Group, "SplineSubscriber Configuration")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineSubscriberConfiguration::m_topic, "Topic", "Topic")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineSubscriberConfiguration::m_allowWGS84, "Allow WGS84", "Allow WGS84")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineSubscriberConfiguration::m_resetOnActivation,
                        "Reset On Activation",
                        "Reset On Activation")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SplineSubscriberConfiguration::m_startOffsetTag,
                        "Start Offset Tag",
                        "Tag that will be used to set the start offset for the spline.");
            }
        }
    }
} // namespace SplineTools