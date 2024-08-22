/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Memory/Memory_fwd.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/base.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <Pointcloud/PointcloudComponentControllerConfigurationBus.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{
    class PointcloudComponentConfig final : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(PointcloudComponentConfig, "{3ae848a0-3cd0-439e-bafe-db0270caae47}");

        PointcloudComponentConfig() = default;
        ~PointcloudComponentConfig() = default;

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_editorEntityId;
        float m_pointSize = 1.0f;
        PointcloudFeatureProcessorInterface::PointcloudHandle m_pointcloudHandle =
            PointcloudFeatureProcessorInterface::InvalidPointcloudHandle;
        AZ::Data::Asset<PointcloudAsset> m_pointcloudAsset;
    };

    class PointcloudComponentController
        : public PointcloudConfigurationBus::Handler
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_TYPE_INFO(PointcloudComponentController, "{b1c3e248-84fc-435f-9983-4a333b859330}");
        PointcloudComponentController() = default;
        explicit PointcloudComponentController(const PointcloudComponentConfig& config);

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        //////////////////////////////////////////////////////////////////////////
        // Controller component
        void Init();
        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const PointcloudComponentConfig& config);
        const PointcloudComponentConfig& GetConfiguration() const;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world);

        // AzToolsFramework::EditorEntityInfoNotificationBus overrides ...
        void OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible);

        // PointcloudEditorComponentConfigurationBus::Handler overrides ...
        void SetPointcloudAsset(AZ::Data::Asset<PointcloudAsset> asset) override;
        void SetPointSize(float pointSize) override;
        void SetVisibility(bool visible) override;

        AZ::Crc32 OnSetPointSize();
        AZ::Crc32 OnAssetChanged();

    private:
        PointcloudFeatureProcessorInterface* m_featureProcessor = nullptr;
        AZ::RPI::Scene* m_scene = nullptr;
        PointcloudComponentConfig m_config;
        bool visibility = true;
    };
} // namespace Pointcloud
