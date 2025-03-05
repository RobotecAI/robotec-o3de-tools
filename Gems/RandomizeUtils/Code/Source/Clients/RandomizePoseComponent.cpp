#include "RandomizePoseComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Entity/GameEntityContextBus.h>
#include <random>
namespace RandomizeUtils
{

    //! Get a sample from a uniform distribution in the range
    float GenerateRandomFloat(float min, float max)
    {
        static std::mt19937 generator(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::uniform_real_distribution<float> distribution(min, max);
        return distribution(generator);
    }

    //! Get a sample for three dimensions from a uniform distribution in the range [-range, range].
    //! The variables are independent.
    //! \param range The range of the random values.
    AZ::Vector3 GenerateRandomVector3(const AZ::Vector3& range)
    {
        static std::mt19937 generator(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));

        std::uniform_real_distribution<float> distributionX(-range.GetX(), range.GetX());
        std::uniform_real_distribution<float> distributionY(-range.GetY(), range.GetY());
        std::uniform_real_distribution<float> distributionZ(-range.GetZ(), range.GetZ());

        return AZ::Vector3(distributionX(generator), distributionY(generator), distributionZ(generator));
    }

    void RandomizePoseComponentConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RandomizePoseComponentConfig>()
                ->Version(1)
                ->Field("PositionRange", &RandomizePoseComponentConfig::m_positionRange)
                ->Field("RotationRange", &RandomizePoseComponentConfig::m_rotationRange)
                ->Field("ScaleRange", &RandomizePoseComponentConfig::m_scaleRange)
                ->Field("ProbabilityOfActivation", &RandomizePoseComponentConfig::m_probabilityOfActivation);
            if (auto* ec = serialize->GetEditContext())
            {
                ec->Class<RandomizePoseComponentConfig>("RandomizePoseComponentConfig", "RandomizePoseComponentConfig")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "RandomizeUtils")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &RandomizePoseComponentConfig::m_positionRange, "Position Range", "Position Range")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &RandomizePoseComponentConfig::m_rotationRange, "Rotation Range", "Rotation Range")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RandomizePoseComponentConfig::m_scaleRange, "Scale Range", "Scale Range")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RandomizePoseComponentConfig::m_probabilityOfActivation,
                        "Probality Of Existance",
                        "Probality Of Existance");
            }
        }
    }

    void RandomizePoseComponent::Reflect(AZ::ReflectContext* context)
    {
        RandomizePoseComponentConfig::Reflect(context);
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RandomizePoseComponent, AZ::Component>()->Version(1)->Field("Config", &RandomizePoseComponent::m_config);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RandomizePoseComponent>("RandomizePoseComponent", "RandomizePoseComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "RandomizeUtils")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RandomizePoseComponent::m_config, "Config", "Config");
            }
        }
    }

    void RandomizePoseComponent::Activate()
    {
        const auto randomPosition = GenerateRandomVector3(m_config.m_positionRange);
        const auto randomRotationDeg = GenerateRandomVector3(m_config.m_rotationRange);
        const float randomScale = 1.0f + GenerateRandomFloat(-m_config.m_scaleRange, m_config.m_scaleRange);
        const auto randomQuat = AZ::Quaternion::CreateFromEulerDegreesXYZ(randomRotationDeg);

        const AZ::Transform transformOffset(randomPosition, randomQuat, randomScale);

        // get local transform
        AZ::Transform localTM = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(localTM, GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);

        // apply random transform to local transform
        localTM = localTM * transformOffset;

        // set local transform
        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetLocalTM, localTM);

        float randomFloat = GenerateRandomFloat(0.0f, 1.0f);
        if (randomFloat > m_config.m_probabilityOfActivation)
        {
            m_entityIsToBeDeactivated = true;
            AZ::EntityBus::Handler::BusConnect(GetEntityId());
        }
    }

    void RandomizePoseComponent::OnEntityActivated(const AZ::EntityId& entityId)
    {
        if (m_entityIsToBeDeactivated)
        {
            AzFramework::GameEntityContextRequestBus::Broadcast(&AzFramework::GameEntityContextRequests::DeactivateGameEntity, entityId);
        }
    }

    void RandomizePoseComponent::Deactivate()
    {
        if (m_entityIsToBeDeactivated)
        {
            AZ::EntityBus::Handler::BusDisconnect();
        }
    }

} // namespace RandomizeUtils
