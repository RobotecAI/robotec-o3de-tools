/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/PipelineState.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <AzCore/Math/Transform.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
namespace Pointcloud
{
    class Scene;
    class Shader;

    class PointcloudFeatureProcessor
        : public PointcloudFeatureProcessorInterface
        , protected AZ::RPI::ViewportContextIdNotificationBus::Handler
        , protected AZ::Data::AssetBus::MultiHandler
    {
    public:
        AZ_RTTI(PointcloudFeatureProcessor, "{B6EF8776-F7F9-432B-8BD9-D43869FFFC3D}", PointcloudFeatureProcessorInterface);
        AZ_CLASS_ALLOCATOR(PointcloudFeatureProcessor, AZ::SystemAllocator)

        static void Reflect(AZ::ReflectContext* context);

        PointcloudFeatureProcessor() = default;
        virtual ~PointcloudFeatureProcessor() = default;

        // PointcloudFeatureProcessorInterface overrides
        void SetTransform(const PointcloudHandle& handle, const AZ::Transform& transform) override;
        void SetPointSize(const PointcloudHandle& handle, float pointSize) override;
        PointcloudHandle AcquirePointcloud(const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData) override;
        void UpdatePointCloud(
            PointcloudHandle PointcloudDataIndex,
            const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData,
            size_t startIdx) override;
        PointcloudHandle AcquirePointcloudFromAsset(AZ::Data::Asset<PointcloudAsset> pointcloudAsset) override;
        void SetVisibility(const PointcloudHandle& handle, bool visible) override;
        void ReleasePointcloud(const PointcloudHandle& handle) override;

    protected:
        // RPI::SceneNotificationBus overrides
        void OnRenderPipelineChanged(
            AZ::RPI::RenderPipeline* pipeline, AZ::RPI::SceneNotification::RenderPipelineChangeType changeType) override;

        // Data::AssetBus overrides
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset) override;

    private:
        struct PointcloudData
        {
            PointcloudHandle m_index = 0;
            AZ::Data::Asset<AZ::RPI::BufferAsset> m_cloudVertexBufferAsset;
            AZ::Data::Instance<AZ::RPI::Buffer> m_cloudVertexBuffer = nullptr;

            AZStd::array<AZ::RHI::StreamBufferView, 1> m_meshStreamBufferViews;
            AZStd::vector<PointcloudAsset::CloudVertex> m_pointData;
            uint32_t m_vertices = 0;
            float m_pointSize = 1.0f;
            AZ::Transform m_transform = AZ::Transform::CreateIdentity();
            AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> m_drawPacket;
            AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> m_drawSrg = nullptr;
            bool m_visible = true;
            bool m_needSrgUpdate = true;
            AZ::Data::AssetId m_assetId; //! AssetId of the pointcloud asset, if pointcloud was acquired from an asset
            AZ::Data::Asset<AZ::Data::AssetData> m_assetData; //! Pointcloud asset data, if pointcloud was acquired from an asset
        };

        // FeatureProcessor overrides
        void Activate() override;
        void Deactivate() override;
        void Simulate(const FeatureProcessor::SimulatePacket& packet) override;
        void Render(const FeatureProcessor::RenderPacket& packet) override;

        void UpdateDrawPacket();
        void UpdateShaderConstants();

        //! build a draw packet to draw the point cloud
        AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> BuildDrawPacket(
            const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup>& srg,
            const AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw>& pipelineState,
            const AZ::RHI::DrawListTag& drawListTag,
            const AZStd::span<const AZ::RHI::StreamBufferView>& streamBufferViews,
            uint32_t vertexCount);

        AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw> m_meshPipelineState;
        AZ::RHI::DrawListTag m_drawListTag;
        AZ::Data::Instance<AZ::RPI::Shader> m_shader = nullptr; //!< Shader for the pointcloud
        AZ::RHI::ShaderInputNameIndex m_pointSizeIndex = "m_pointSize";
        AZ::RHI::ShaderInputNameIndex m_modelMatrixIndex = "m_modelMatrix";
        AZ::RHI::Ptr<AZ::RHI::ShaderResourceGroupLayout> m_drawSrgLayout; //!< Shader resource group layout for the draw packet
        AZStd::unordered_map<PointcloudHandle, PointcloudData> m_pointcloudData; //!< Map of pointcloud data
        PointcloudHandle m_currentPointcloudDataIndex = 0; //!< Index to the next pointcloud data to be created
        AZStd::unordered_map<AZ::Data::AssetId, PointcloudHandle> m_pointcloudAssets;
    };
} // namespace Pointcloud