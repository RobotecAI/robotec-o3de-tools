#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <AzCore/Component/TransformBus.h>
#include <Billboard/BillboardFeatureProcessorInterface.h>
#include <AzFramework/Scene/Scene.h>
namespace Billboard
{

    class BillboardEditorComponent : public AzToolsFramework::Components::EditorComponentBase,
        private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(BillboardEditorComponent, "{018fba15-560f-78cb-afb4-cf4d00cefc11}");
        BillboardEditorComponent() = default;
        ~BillboardEditorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:

        //AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        AZ::Crc32 OnSetPointSize();
        AZ::Crc32 LoadCloud();
        float m_pointSize = 1.0f;
        bool m_moveToCentroid {true};
        BillboardFeatureProcessorInterface *m_featureProcessor = nullptr;
        AZ::RPI::Scene *m_scene = nullptr;
    };
} // namespace Billboard
