#include "SmoothingComponent.h"
#include "Utils.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
namespace Smoothing
{
    void SmoothingConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SmoothingConfig, AZ::ComponentConfig>()
                ->Version(1)
                ->Field("m_entityToTrack", &SmoothingConfig::m_entityToTrack)
                ->Field("m_lockZAxis", &SmoothingConfig::m_lockZAxis)
                ->Field("m_smoothBufferLen", &SmoothingConfig::m_smoothBufferLen)
                ->Field("m_smoothingMethod", &SmoothingConfig::m_smoothingMethod)
                ->Field("m_dampingFactor", &SmoothingConfig::m_dampingFactor)
                ->Field("m_springFactor", &SmoothingConfig::m_springFactor);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SmoothingConfig>("SmoothingConfig", "An example of a component that does something")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SmoothingConfig::m_entityToTrack,
                        "Entity to Track",
                        "The entity to track for smoothing")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SmoothingConfig::m_lockZAxis,
                        "Lock Z Axis",
                        "Lock the Z axis of the entity to track")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &SmoothingConfig::OnSmoothMethodChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &SmoothingConfig::m_smoothingMethod,
                        "Smoothing method",
                        "A method to smooth the entity's position")
                    ->EnumAttribute(SmoothingConfig::SmoothingAlgorithm::NoSmoothing, "No Smoothing")
                    ->EnumAttribute(SmoothingConfig::SmoothingAlgorithm::AverageSmoothing, "Average Smoothing")
                    ->EnumAttribute(SmoothingConfig::SmoothingAlgorithm::DampingSmoothing, "Damping Smoothing")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &SmoothingConfig::OnSmoothMethodChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SmoothingConfig::m_smoothBufferLen,
                        "Smooth Buffer Length",
                        "The length of the buffer to smooth the entity's position")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &SmoothingConfig::SmoothBufferVisibility)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SmoothingConfig::m_dampingFactor,
                        "Damping Factor",
                        "The damping factor to smooth the entity's position")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &SmoothingConfig::DampingFactorVisibility)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SmoothingConfig::m_springFactor,
                        "Spring Factor",
                        "The spring factor to smooth the entity's position")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &SmoothingConfig::DampingFactorVisibility)
                    ->UIElement(AZ::Edit::UIHandlers::Label, "", "")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(
                        AZ::Edit::Attributes::ValueText, "It is recommended to use damping smoothing with lock Z axis for better results.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &SmoothingConfig::DampingFactorVisibilityWarning);
            }
        }
    }

    AZ::Crc32 SmoothingConfig::OnSmoothMethodChanged()
    {
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::Crc32 SmoothingConfig::SmoothBufferVisibility() const
    {
        return m_smoothingMethod == SmoothingAlgorithm::AverageSmoothing ? AZ::Edit::PropertyVisibility::Show
                                                                         : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 SmoothingConfig::DampingFactorVisibility() const
    {
        return m_smoothingMethod == SmoothingAlgorithm::DampingSmoothing ? AZ::Edit::PropertyVisibility::Show
                                                                         : AZ::Edit::PropertyVisibility::Hide;
    }
    AZ::Crc32 SmoothingConfig::DampingFactorVisibilityWarning() const
    {
        return m_smoothingMethod == SmoothingAlgorithm::DampingSmoothing && !m_lockZAxis ? AZ::Edit::PropertyVisibility::Show
                                                                                         : AZ::Edit::PropertyVisibility::Hide;
    }

    void SmoothingComponentController::Reflect(AZ::ReflectContext* context)
    {
        SmoothingConfig::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SmoothingComponentController>()
                ->Field("Configuration", &SmoothingComponentController::m_config)
                ->Version(1);
            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<SmoothingComponentController>("SmoothingComponentController", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Smoothing")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SmoothingComponentController::m_config);
            }
        }
    }

    SmoothingComponentController::SmoothingComponentController(const SmoothingConfig& config)
        : m_config(config)
    {
    }

    void SmoothingComponentController::Activate(AZ::EntityId entityId)
    {
        m_ourLastVelocity = AZ::Vector3::CreateZero();
        m_ourLastAngularVelocity = AZ::Vector3::CreateZero();
        m_entityId = entityId;
        m_lastTargetTransform.reset();
        AZ::TickBus::Handler::BusConnect();
    }

    void SmoothingComponentController::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void SmoothingComponentController::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        AZ_UNUSED(time);
        if (!m_smoothingEnabled)
        {
            return;
        }
        // get transform of entity to track
        AZ::Transform targetTransform = AZ::Transform::CreateIdentity();
        AZ::Transform ourTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(targetTransform, m_config.m_entityToTrack, &AZ::TransformBus::Events::GetWorldTM);
        AZ::TransformBus::EventResult(ourTransform, m_entityId, &AZ::TransformBus::Events::GetWorldTM);

        AZ::Transform newTransform = AZ::Transform::CreateIdentity();
        // lock Z axis if needed
        if (m_config.m_lockZAxis)
        {
            targetTransform = SmoothingUtils::RemoveTiltFromTransform(targetTransform);
        }

        if (m_config.m_smoothingMethod == SmoothingConfig::SmoothingAlgorithm::NoSmoothing)
        {
            newTransform = targetTransform;
        }
        else if (m_config.m_smoothingMethod == SmoothingConfig::SmoothingAlgorithm::AverageSmoothing)
        {
            if (m_config.m_smoothBufferLen > 0)
            {
                SmoothingUtils::CacheTransform(m_smoothingCache, targetTransform, deltaTime, m_config.m_smoothBufferLen);
                const auto positionSmooth = SmoothingUtils::SmoothTranslation(m_smoothingCache);
                newTransform.SetTranslation(positionSmooth);
                const auto rotationSmooth = SmoothingUtils::SmoothRotation(m_smoothingCache);
                newTransform.SetRotation(rotationSmooth);
            }
        }
        else if (m_config.m_smoothingMethod == SmoothingConfig::SmoothingAlgorithm::DampingSmoothing)
        {
            // clip delta time to avoid big jumps
            deltaTime = AZStd::min(deltaTime, 0.1f);
            if (m_isFirstTick)
            {
                m_lastTargetTransform = targetTransform;
                newTransform = targetTransform;
            }
            if (m_ourLastTransform.has_value() && m_lastTargetTransform.has_value())
            {
                // easy part - linear damping
                {
                    const AZ::Vector3 targetPosition = targetTransform.GetTranslation();
                    const AZ::Vector3 ourPosition = ourTransform.GetTranslation();

                    // simplified mass-spring-damper model where damping is proportional to our velocity
                    const AZ::Vector3 force =
                        m_config.m_springFactor * (targetPosition - ourPosition) - m_config.m_dampingFactor * m_ourLastVelocity;
                    m_ourLastVelocity += force * deltaTime;
                    newTransform.SetTranslation(ourPosition + m_ourLastVelocity * deltaTime);
                }
                // harder part - rotation
                {
                    // The cross product of two vectors is a vector that is perpendicular to both vector, so it can be used to calculate the
                    // torque response.
                    // We compute sum of cross products of target and our direction X and Y vectors.
                    // We need two cross products to get the torque response in all three axes.
                    // We can imagine a system as three rubbers bands that attached to the target and our entity and we are trying to align
                    // them. The first is attached to the target and our entity's X axis, the second is attached to the target and our
                    // entity's Y axis.
                    auto diff = ourTransform.GetBasisX().Cross(targetTransform.GetBasisX()) +
                        ourTransform.GetBasisY().Cross(targetTransform.GetBasisY());

                    const AZ::Vector3 torque = m_config.m_springFactor * (diff)-m_config.m_dampingFactor * m_ourLastAngularVelocity;
                    m_ourLastAngularVelocity += torque * deltaTime;
                    // to be on safe side we lock Z axis for velocity
                    if (m_config.m_lockZAxis)
                    {
                        m_ourLastAngularVelocity.SetX(.0f);
                        m_ourLastAngularVelocity.SetY(.0f);
                    }
                    newTransform.SetRotation(
                        ourTransform.GetRotation() * AZ::Quaternion::CreateFromScaledAxisAngle(m_ourLastAngularVelocity * deltaTime));
                }
            }
        }
        m_isFirstTick = false;
        m_lastTargetTransform = targetTransform;
        m_ourLastTransform = newTransform;
        AZ::TransformBus::Event(m_entityId, &AZ::TransformBus::Events::SetWorldTM, newTransform);
    }

    int SmoothingComponentController::GetTickOrder()
    {
        return AZ::TICK_DEFAULT;
    }

    void SmoothingComponentController::Init()
    {
    }

    void SmoothingComponentController::SetConfiguration(const SmoothingConfig& config)
    {
        m_config = config;
    }

    const SmoothingConfig& SmoothingComponentController::GetConfiguration() const
    {
        return m_config;
    }

    SmoothingComponent::SmoothingComponent(const SmoothingConfig& config)
        : SmoothingComponentBase(config)
    {
    }

    void SmoothingComponent::Reflect(AZ::ReflectContext* context)
    {
        SmoothingComponentBase::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SmoothingComponent, SmoothingComponentBase>()->Version(1);
        }
    }

    void SmoothingComponent::Activate()
    {
        SmoothingComponentBase::Activate();
    }

    void SmoothingComponent::Deactivate()
    {
        SmoothingComponentBase::Deactivate();
    }

    void SmoothingComponentController::SetSmoothingEnabled(bool enabled)
    {
        m_smoothingEnabled = enabled;
    }

    bool SmoothingComponentController::GetSmoothingEnabled() const
    {
        return m_smoothingEnabled;
    }
} // namespace Smoothing
