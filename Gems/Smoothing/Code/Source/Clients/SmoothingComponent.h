#pragma once

#include "Utils.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/optional.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include <Smoothing/SmoothingTypeIds.h>

namespace Smoothing
{
    struct SmoothingConfig : public AZ::ComponentConfig
    {
        AZ_RTTI(SmoothingConfig, SmoothingConfigTypeId, AZ::ComponentConfig);

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_entityToTrack;

        enum class SmoothingAlgorithm : AZ::u8
        {
            NoSmoothing = 0,
            AverageSmoothing,
            DampingSmoothing,
        };

        SmoothingAlgorithm m_smoothingMethod = SmoothingAlgorithm::DampingSmoothing;
        bool m_lockZAxis = false;

        // config for average smoothing
        int m_smoothBufferLen = 10;

        // config for damping smoothing
        float m_dampingFactor = 10.f;
        float m_springFactor = 10.f;

    private:
        AZ::Crc32 SmoothBufferVisibility() const;
        AZ::Crc32 DampingFactorVisibility() const;
        AZ::Crc32 DampingFactorVisibilityWarning() const;

        AZ::Crc32 OnSmoothMethodChanged();
    };

    class SmoothingComponentController : public AZ::TickBus::Handler
    {
    public:
        AZ_TYPE_INFO(SmoothingComponentController, SmoothingComponentControllerTypeId);

        static void Reflect(AZ::ReflectContext* context);

        SmoothingComponentController() = default;
        SmoothingComponentController(const SmoothingConfig& config);

        // Controller component ...
        void Init();
        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const SmoothingConfig& config);
        const SmoothingConfig& GetConfiguration() const;

        // TickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        int GetTickOrder() override;

    protected:
        SmoothingConfig m_config;
        SmoothingUtils::SmoothingCache m_smoothingCache;
        AZ::EntityId m_entityId;
        AZStd::optional<AZ::Transform> m_lastTargetTransform;
        AZStd::optional<AZ::Transform> m_ourLastTransform;
        AZ::Vector3 m_ourLastVelocity;
        AZ::Vector3 m_ourLastAngularVelocity;
        bool m_isFirstTick = true;
    };
    using SmoothingComponentBase = AzFramework::Components::ComponentAdapter<SmoothingComponentController, SmoothingConfig>;

    class SmoothingComponent : public SmoothingComponentBase
    {
    public:
        AZ_COMPONENT(SmoothingComponent, SmoothingComponentTypeId, AZ::Component);
        static void Reflect(AZ::ReflectContext* context);

        SmoothingComponent(const SmoothingConfig& config);
        SmoothingComponent() = default;
        ~SmoothingComponent() = default;

        // Component overrides...
        void Activate() override;
        void Deactivate() override;
    };
} // namespace Smoothing
