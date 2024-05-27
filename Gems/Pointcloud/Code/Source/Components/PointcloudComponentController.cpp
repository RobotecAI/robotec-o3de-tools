/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Components/PointcloudComponentController.h>

#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityContext.h>
#include <AzFramework/Scene/Scene.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <AzCore/RTTI/BehaviorContext.h>

#include <Atom/RPI.Public/Scene.h>
#include "../Render/PointcloudFeatureProcessor.h"
#include <AzCore/Math/Transform.h>
#include <3rd/happly.h>
namespace Pointcloud {
    void PointcloudComponentConfig::Reflect(AZ::ReflectContext *context) {
        if (auto *serializeContext = azrtti_cast<AZ::SerializeContext *>(context)) {
            serializeContext->Class<PointcloudComponentConfig>()
                    ->Version(0)
                    ->Field("Pointsize", &PointcloudComponentConfig::m_pointSize)
                    ->Field("plyFilePath", &PointcloudComponentConfig::m_plyFilePath);
            if (AZ::EditContext *editContext = serializeContext->GetEditContext()) {
                editContext->Class<PointcloudComponentConfig>(
                                "PointcloudComponentConfig", "")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &PointcloudComponentConfig::m_pointSize,
                                      "Pointsize", "")
                                      ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                                      ->Attribute(AZ::Edit::Attributes::Max, 10.0f)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &PointcloudComponentConfig::m_plyFilePath,
                                        "plyFilePath", "");

            }
        }
    }

    void PointcloudComponentController::Reflect(AZ::ReflectContext *context) {
        PointcloudComponentConfig::Reflect(context);

        if (auto *serializeContext = azrtti_cast<AZ::SerializeContext *>(context)) {
            serializeContext->Class<PointcloudComponentController>()
                    ->Version(0)
                    ->Field("Configuration", &PointcloudComponentController::m_configuration);

            if (AZ::EditContext *editContext = serializeContext->GetEditContext()) {
                editContext->Class<PointcloudComponentController>(
                                "PointcloudComponentController", "")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &PointcloudComponentController::m_configuration,
                                      "Configuration", "")
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudComponentController::OnConfigurationChanged)
                        ->UIElement(AZ::Edit::UIHandlers::Button, "Load" , "Load")
                        ->Attribute(AZ::Edit::Attributes::ButtonText, "Load")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &PointcloudComponentController::OnLoadButton)
            }
        }
    }

    void PointcloudComponentController::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType &dependent) {
        dependent.push_back(AZ_CRC_CE("TransformService"));
    }

    void PointcloudComponentController::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType &provided) {
        provided.push_back(AZ_CRC_CE("PointcloudService"));
    }

    void
    PointcloudComponentController::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType &incompatible) {
        incompatible.push_back(AZ_CRC_CE("PointcloudService"));
    }

    void PointcloudComponentController::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType &required) {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    PointcloudComponentController::PointcloudComponentController(const PointcloudComponentConfig &config)
            : m_configuration(config) {
    }

    void PointcloudComponentController::Activate(AZ::EntityId entityId) {
        m_entityId = entityId;
        m_scene = AZ::RPI::Scene::GetSceneForEntityId(entityId);
        if (m_scene) {
            m_featureProcessor = m_scene->EnableFeatureProcessor<PointcloudFeatureProcessor>();

            AZ_Assert(m_featureProcessor, "Failed to enable PointcloudFeatureProcessorInterface.");

        }

        AZ::TransformNotificationBus::Handler::BusConnect(m_entityId);

    }

    void PointcloudComponentController::Deactivate() {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
    }

    void PointcloudComponentController::SetConfiguration(const PointcloudComponentConfig &config) {
        m_configuration = config;
    }

    const PointcloudComponentConfig &PointcloudComponentController::GetConfiguration() const {
        return m_configuration;
    }

    void PointcloudComponentController::OnTransformChanged([[maybe_unused]] const AZ::Transform &local,
                                                           [[maybe_unused]] const AZ::Transform &world) {
        if (!m_featureProcessor) {
            return;
        }
        m_featureProcessor->SetTransform(world);

    }

    AZ::Crc32 PointcloudComponentController::OnConfigurationChanged()
    {
        if (!m_featureProcessor) {
            return AZ::Edit::PropertyRefreshLevels::None;
        }
        m_featureProcessor->SetPointSize(m_configuration.m_pointSize);
        return AZ::Edit::PropertyRefreshLevels::None;
    }
    AZ::Crc32 PointcloudComponentController::OnLoadButton()
    {
        AZStd::string path = m_configuration.m_plyFilePath;
        AZ_Printf("PointcloudComponentController", "Loading file: %s", path.c_str());
        happly::PLYData plyIn(path.c_str());
    }

}
