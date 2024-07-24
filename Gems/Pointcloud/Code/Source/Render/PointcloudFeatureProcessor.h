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
        , protected AZ::Data::AssetBus::Handler
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
        void SetVisibility(const PointcloudHandle& handle, bool visible) override;
        void ReleasePointcloud(const PointcloudHandle& handle) override;

    protected:
        // RPI::SceneNotificationBus overrides
        void OnRenderPipelineChanged(
            AZ::RPI::RenderPipeline* pipeline, AZ::RPI::SceneNotification::RenderPipelineChangeType changeType) override;

        // Data::AssetBus overrides
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;

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
            bool m_visible = true;
            bool m_needSrgUpdate = true;
        };
        void UpdateBackgroundClearColor();

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
        AZ::Data::Instance<AZ::RPI::Shader> m_shader = nullptr;
        AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> m_drawSrg = nullptr;

        AzFramework::WindowSize m_viewportSize{ 0, 0 };

        AZ::RHI::ShaderInputNameIndex m_pointSizeIndex = "m_pointSize";
        AZ::RHI::ShaderInputNameIndex m_modelMatrixIndex = "m_modelMatrix";

        AZStd::unordered_map<PointcloudHandle, PointcloudData> m_pointcloudData;
        PointcloudHandle m_currentPointcloudDataIndex = 0;
    };
} // namespace Pointcloud