#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>
#include <RandomizeUtils/RandomizeUtilsTypeIds.h>

namespace RandomizeUtils
{
    struct RandomizePoseComponentConfig
    {
        AZ_TYPE_INFO(RandomizePoseComponentConfig, "{9c8e491e-6de5-4009-8c8f-874b7ff14200}");
        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 m_positionRange = AZ::Vector3(0.0f, 0.0f, 0.0f);
        AZ::Vector3 m_rotationRange = AZ::Vector3(0.0f, 0.0f, 0.0f);
        float m_scaleRange = 0.0f;
        float m_probabilityOfActivation = 1.0f;
        // Add your component properties here
    };

    class RandomizePoseComponent
        : public AZ::Component
        , public AZ::EntityBus::Handler
    {
    public:
        AZ_COMPONENT(RandomizePoseComponent, RandomizePoseComponentTypeId);
        static void Reflect(AZ::ReflectContext* context);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
        {
            required.push_back(AZ_CRC_CE("TransformService"));
        }
        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::EntityBus::Handler interface implementation
        void OnEntityActivated(const AZ::EntityId& entityId) override;

    private:
        RandomizePoseComponentConfig m_config;
        bool m_entityIsToBeDeactivated = false;
    };
} // namespace RandomizeUtils
