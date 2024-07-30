#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include "Utils.h"
struct SmoothingConfig
    : public AZ::ComponentConfig
{
    AZ_RTTI(SmoothingConfig, "{01910429-f597-7e0b-a4a2-8ac297a0d5ed}", AZ::ComponentConfig);

    static void Reflect(AZ::ReflectContext* context);

    AZ::EntityId m_entityToTrack;
    bool m_lockZAxis = false;
    int m_smoothBufferLen = 10;
};

class SmoothingComponentController:
     public AZ::TickBus::Handler
{
public:
    AZ_TYPE_INFO(SmoothingComponentController, "{F8A7B6D3-56E2-4B6E-A869-5F16B5E3A4D4}");

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
};
using SmoothingComponentBase = AzFramework::Components::ComponentAdapter<SmoothingComponentController, SmoothingConfig>;


class SmoothingComponent : public SmoothingComponentBase
{
public:
    AZ_COMPONENT(SmoothingComponent, "{0191042c-ff02-760f-b49b-60f03db706ff}", AZ::Component);
    static void Reflect(AZ::ReflectContext* context);

    SmoothingComponent(const SmoothingConfig& config);
    SmoothingComponent() = default;
    ~SmoothingComponent() = default;

    // Component overrides...
    void Activate() override;
    void Deactivate() override;


};

