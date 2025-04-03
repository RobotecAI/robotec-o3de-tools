/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudFeatureProcessor.h"
#include <Atom/RHI.Reflect/InputStreamLayoutBuilder.h>
#include <Atom/RHI/DrawPacketBuilder.h>
#include <Atom/RPI.Public/Pass/PassFilter.h>
#include <Atom/RPI.Public/RPIUtils.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <Atom/RPI.Reflect/Buffer/BufferAssetCreator.h>
#include <Atom/RPI.Reflect/ResourcePoolAssetCreator.h>
#include <AzCore/Name/NameDictionary.h>
#include <fstream>

namespace Pointcloud
{
    void PointcloudFeatureProcessor::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudFeatureProcessor, FeatureProcessor>();
        }
    }

    void PointcloudFeatureProcessor::Activate()
    {
        const char* shaderFilePath = "Shaders/Pointclouds/Pointclouds.azshader";
        m_shader = AZ::RPI::LoadCriticalShader(shaderFilePath);

        if (!m_shader)
        {
            AZ_Error("PointcloudFeatureProcessor", false, "Failed to load required pointcloud shader.");
            return;
        }
        AZ::Data::AssetBus::MultiHandler::BusConnect(m_shader->GetAssetId());

        m_drawSrgLayout = m_shader->GetAsset()->GetDrawSrgLayout(m_shader->GetSupervariantIndex());
        AZ_Error(
            "PointcloudFeatureProcessor",
            m_drawSrgLayout,
            "Failed to get the draw shader resource group layout for the pointcloud shader.");

        m_drawListTag = m_shader->GetDrawListTag();

        auto viewportContextInterface = AZ::Interface<AZ::RPI::ViewportContextRequestsInterface>::Get();
        AZ_Assert(viewportContextInterface, "PointcloudFeatureProcessor requires the ViewportContextRequestsInterface.");
        auto viewportContext = viewportContextInterface->GetViewportContextByScene(GetParentScene());
        AZ_Assert(viewportContext, "PointcloudFeatureProcessor requires a valid ViewportContext.");

        EnableSceneNotification();

        AZ::RPI::ViewportContextIdNotificationBus::Handler::BusConnect(viewportContext->GetId());
    }

    PointcloudFeatureProcessorInterface::PointcloudHandle PointcloudFeatureProcessor::AcquirePointcloud(
        const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData)
    {
        m_currentPointcloudDataIndex++;
        auto& pcData = m_pointcloudData[m_currentPointcloudDataIndex];
        pcData.m_index = m_currentPointcloudDataIndex;
        UpdatePointCloud(m_currentPointcloudDataIndex, cloudVertexData, 0);
        return pcData.m_index;
    }

    void PointcloudFeatureProcessor::UpdatePointCloud(
        PointcloudHandle PointcloudDataIndex, const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData, size_t startIdx)
    {
        if (m_pointcloudData.find(PointcloudDataIndex) == m_pointcloudData.end())
        {
            AZ_Error(
                "PointcloudFeatureProcessor",
                false,
                "PointcloudDataIndex not found, use AcquirePointcloud to create a new pointcloud first");
            return;
        }
        AZ_Assert(m_drawSrgLayout, "DrawSrgLayout is not initialized");
        auto& pcData = m_pointcloudData[PointcloudDataIndex];
        constexpr uint32_t elementSize = sizeof(PointcloudAsset::CloudVertex);

        const uint32_t elementCount = AZStd::max(static_cast<uint32_t>(cloudVertexData.size() + startIdx), pcData.m_vertices);
        // new element count
        const uint32_t bufferSize = elementCount * elementSize; // bytecount

        pcData.m_pointData.resize(bufferSize);
        pcData.m_vertices = elementCount;
        memcpy(pcData.m_pointData.data() + startIdx * elementSize, cloudVertexData.data(), cloudVertexData.size() * elementSize);

        if (pcData.m_cloudVertexBuffer)
        {
            pcData.m_cloudVertexBuffer->Resize(bufferSize);
            pcData.m_cloudVertexBuffer->UpdateData(pcData.m_pointData.data(), bufferSize, startIdx * elementSize);
        }
        else
        {
            AZ::RPI::CommonBufferDescriptor desc;
            desc.m_poolType = AZ::RPI::CommonBufferPoolType::DynamicInputAssembly;
            desc.m_bufferName = AZStd::string::format("PointcloudFeatureProcessor, %d", pcData.m_index);
            desc.m_byteCount = bufferSize;
            desc.m_elementSize = elementSize;
            desc.m_bufferData = pcData.m_pointData.data();

            pcData.m_cloudVertexBuffer = AZ::RPI::BufferSystemInterface::Get()->CreateBufferFromCommonPool(desc);
        }
        pcData.m_meshStreamBufferViews.front() =
            AZ::RHI::StreamBufferView(*pcData.m_cloudVertexBuffer->GetRHIBuffer(), 0, bufferSize, elementSize);

        if (!pcData.m_drawSrg)
        {
            pcData.m_drawSrg =
                AZ::RPI::ShaderResourceGroup::Create(m_shader->GetAsset(), m_shader->GetSupervariantIndex(), m_drawSrgLayout->GetName());
            m_pointSizeIndex.Reset();
            m_modelMatrixIndex.Reset();
        }

        // Update bounds
        AZ::Aabb aabb = AZ::Aabb::CreateNull();
        for (const auto& vertex : cloudVertexData)
        {
            AZ::Vector3 position = AZ::Vector3(vertex.m_position[0], vertex.m_position[1], vertex.m_position[2]);
            aabb.AddPoint(position);
        }
        pcData.m_bounds = aabb;

        UpdateDrawPacket();
        m_pointcloudChangedEvent.Signal(PointcloudDataIndex);
    }

    PointcloudFeatureProcessorInterface::PointcloudHandle PointcloudFeatureProcessor::AcquirePointcloudFromAsset(
        AZ::Data::Asset<PointcloudAsset> pointcloudAsset)
    {
        m_currentPointcloudDataIndex++;
        auto& pcData = m_pointcloudData[m_currentPointcloudDataIndex];
        pcData.m_index = m_currentPointcloudDataIndex;
        m_pointcloudAssets[pointcloudAsset.GetId()] = m_currentPointcloudDataIndex;
        pcData.m_assetId = pointcloudAsset.GetId();
        AZ::Data::AssetBus::MultiHandler::BusConnect(pointcloudAsset.GetId());
        if (!(pointcloudAsset.IsLoading() || pointcloudAsset.IsReady()))
        {
            pointcloudAsset.QueueLoad();
        }
        if (pointcloudAsset.IsReady())
        {
            OnAssetReady(pointcloudAsset);
        }
        return m_currentPointcloudDataIndex;
    }

    void PointcloudFeatureProcessor::UpdateDrawPacket()
    {
        for (auto& [_, pcData] : m_pointcloudData)
        {
            if (m_meshPipelineState && pcData.m_drawSrg && pcData.m_meshStreamBufferViews.front().GetByteCount() != 0)
            {
                pcData.m_drawPacket = BuildDrawPacket(
                    pcData.m_drawSrg, m_meshPipelineState, m_drawListTag, pcData.m_meshStreamBufferViews, pcData.m_geometryView, pcData.m_vertices);
            }
        }
    }

    void PointcloudFeatureProcessor::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset.GetId() == m_shader->GetAssetId())
        {
            UpdateDrawPacket();
        }
        else if (auto it = m_pointcloudAssets.find(asset.GetId()); it != m_pointcloudAssets.end())
        {
            AZ_Printf(
                "PointcloudFeatureProcessor",
                "Pointcloud asset reloaded,  old %s, new : %s",
                it->first.ToString<AZStd::string>().c_str(),
                asset.GetId().ToString<AZStd::string>().c_str());
            const auto handle = it->second;
            auto& pcData = m_pointcloudData[handle];
            auto newAssetData = asset.GetAs<PointcloudAsset>();
            AZ_Assert(newAssetData, "Asset is not of the expected type.");
            pcData.m_assetData = asset; // store asset to have ref count of asset
            UpdatePointCloud(handle, newAssetData->m_data, 0);
        }
    }

    void PointcloudFeatureProcessor::OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        OnAssetReloaded(asset);
    }

    void PointcloudFeatureProcessor::Deactivate()
    {
        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor Deactivated");
        AZ::Data::AssetBus::MultiHandler::BusDisconnect(m_shader->GetAssetId());
        for (const auto& [assetId, _] : m_pointcloudAssets)
        {
            AZ::Data::AssetBus::MultiHandler::BusDisconnect(assetId);
        }
        AZ::RPI::ViewportContextIdNotificationBus::Handler::BusDisconnect();
    }

    void PointcloudFeatureProcessor::Simulate([[maybe_unused]] const FeatureProcessor::SimulatePacket& packet)
    {
        UpdateShaderConstants();
    }

    void PointcloudFeatureProcessor::Render([[maybe_unused]] const FeatureProcessor::RenderPacket& packet)
    {
        AZ_PROFILE_FUNCTION(AzRender);

        for (auto& [_, pcData] : m_pointcloudData)
        {
            if (pcData.m_drawPacket)
            {
                if (!pcData.m_visible)
                {
                    continue;
                }
                for (auto& view : packet.m_views)
                {
                    if (!view->HasDrawListTag(m_drawListTag))
                    {
                        continue;
                    }
                    constexpr float depth = 0.f;
                    view->AddDrawPacket(pcData.m_drawPacket.get(), depth);
                }
            }
        }
    }

    AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> PointcloudFeatureProcessor::BuildDrawPacket(
        const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup>& srg,
        const AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw>& pipelineState,
        const AZ::RHI::DrawListTag& drawListTag,
        const AZStd::span<const AZ::RHI::StreamBufferView>& streamBufferViews,
        AZ::RHI::GeometryView& geometryView,
        uint32_t vertexCount)
    {
        geometryView.Reset();
        AZ::RHI::DrawLinear drawLinear;
        drawLinear.m_vertexCount = vertexCount;
        drawLinear.m_vertexOffset = 0;
        AZ::RHI::DrawInstanceArguments drawInstanceArgs;
        drawInstanceArgs.m_instanceCount = 1;
        drawInstanceArgs.m_instanceOffset = 0;
        geometryView.SetDrawArguments(drawLinear);
        
        for (size_t i = 0; i < streamBufferViews.size(); ++i)
        {
            geometryView.AddStreamBufferView(streamBufferViews[i]);
        }
        AZ::RHI::DrawPacketBuilder drawPacketBuilder{ AZ::RHI::MultiDevice::AllDevices };
        drawPacketBuilder.Begin(nullptr);
        drawPacketBuilder.SetGeometryView(&geometryView);
        drawPacketBuilder.SetDrawInstanceArguments(drawInstanceArgs);
        drawPacketBuilder.AddShaderResourceGroup(srg->GetRHIShaderResourceGroup());

        AZ::RHI::DrawPacketBuilder::DrawRequest drawRequest;
        drawRequest.m_listTag = drawListTag;
        drawRequest.m_pipelineState = pipelineState->GetRHIPipelineState();
        drawRequest.m_streamIndices = geometryView.GetFullStreamBufferIndices();

        drawPacketBuilder.AddDrawItem(drawRequest);
        return drawPacketBuilder.End();
    }

    void PointcloudFeatureProcessor::OnRenderPipelineChanged(
        [[maybe_unused]] AZ::RPI::RenderPipeline* renderPipeline, AZ::RPI::SceneNotification::RenderPipelineChangeType changeType)
    {
        if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::Added)
        {
            if (!m_meshPipelineState)
            {
                m_meshPipelineState = aznew AZ::RPI::PipelineStateForDraw;
                m_meshPipelineState->Init(m_shader);

                AZ::RHI::InputStreamLayoutBuilder layoutBuilder;
                layoutBuilder.AddBuffer()
                    ->Channel("POSITION", AZ::RHI::Format::R32G32B32_FLOAT)
                    ->Channel("COLOR", AZ::RHI::Format::R8G8B8A8_UNORM);
                layoutBuilder.SetTopology(AZ::RHI::PrimitiveTopology::PointList);
                auto inputStreamLayout = layoutBuilder.End();

                m_meshPipelineState->SetInputStreamLayout(inputStreamLayout);
                m_meshPipelineState->SetOutputFromScene(GetParentScene());
                m_meshPipelineState->Finalize();

                UpdateDrawPacket();
            }
        }
        else if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::PassChanged)
        {
            if (m_meshPipelineState)
            {
                m_meshPipelineState->SetOutputFromScene(GetParentScene());
                m_meshPipelineState->Finalize();
                UpdateDrawPacket();
            }
        }
    }

    void PointcloudFeatureProcessor::UpdateShaderConstants()
    {
        for (auto& [_, pcData] : m_pointcloudData)
        {
            if (pcData.m_needSrgUpdate && pcData.m_drawSrg)
            {
                pcData.m_needSrgUpdate = false;
                AZ::Matrix4x4 orientation = AZ::Matrix4x4::CreateFromTransform(pcData.m_transform);
                pcData.m_drawSrg->SetConstant(m_modelMatrixIndex, orientation);
                pcData.m_drawSrg->SetConstant(m_pointSizeIndex, pcData.m_pointSize);
                pcData.m_drawSrg->Compile();
            }
        }
    }

    void PointcloudFeatureProcessor::SetTransform(const PointcloudHandle& handle, const AZ::Transform& transform)
    {
        if (auto it = m_pointcloudData.find(handle); it != m_pointcloudData.end())
        {
            it->second.m_transform = transform;
            it->second.m_needSrgUpdate = true;
        }
    }

    void PointcloudFeatureProcessor::SetPointSize(const PointcloudHandle& handle, float pointSize)
    {
        if (auto it = m_pointcloudData.find(handle); it != m_pointcloudData.end())
        {
            it->second.m_pointSize = pointSize;
            it->second.m_needSrgUpdate = true;
        }
    }

    void PointcloudFeatureProcessor::SetVisibility(const PointcloudHandle& handle, bool visible)
    {
        if (auto it = m_pointcloudData.find(handle); it != m_pointcloudData.end())
        {
            it->second.m_visible = visible;
        }
    }

    void PointcloudFeatureProcessor::ReleasePointcloud(const PointcloudHandle& handle)
    {
        if (auto it = m_pointcloudData.find(handle); it != m_pointcloudData.end())
        {
            m_pointcloudData.erase(it);
        }

        for (auto it = m_pointcloudAssets.begin(); it != m_pointcloudAssets.end(); ++it)
        {
            if (it->second == handle)
            {
                AZ::Data::AssetBus::MultiHandler::BusDisconnect(it->first);
                m_pointcloudAssets.erase(it);
                break;
            }
        }
    }
    uint32_t PointcloudFeatureProcessor::GetPointCount(const PointcloudHandle& handle) const
    {
        if (auto it = m_pointcloudData.find(handle); it != m_pointcloudData.end())
        {
            return it->second.m_vertices;
        }
        return 0;
    }

    AZStd::optional<AZ::Aabb> PointcloudFeatureProcessor::GetBounds(const PointcloudHandle& handle) const
    {
        if (auto it = m_pointcloudData.find(handle); it != m_pointcloudData.end())
        {
            return it->second.m_bounds;
        }
        return AZStd::nullopt;
    }
    
    void PointcloudFeatureProcessor::ConnectChangeEventHandler(
        const PointcloudHandle& pointcloudHandle, PointcloudChangedEvent::Handler& handler)
    {
        if (pointcloudHandle != InvalidPointcloudHandle)
        {
            handler.Connect(m_pointcloudChangedEvent);
        }
    }
} // namespace Pointcloud
