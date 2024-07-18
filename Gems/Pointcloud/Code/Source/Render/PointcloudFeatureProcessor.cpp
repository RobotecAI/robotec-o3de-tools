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
        AZ_UNUSED(m_resourcePoolAsset);
#if 1
        auto bufferPoolDesc = AZStd::make_unique<AZ::RHI::BufferPoolDescriptor>();
        bufferPoolDesc->m_bindFlags = AZ::RHI::BufferBindFlags::InputAssembly;
        bufferPoolDesc->m_heapMemoryLevel = AZ::RHI::HeapMemoryLevel::Device;
        bufferPoolDesc->m_budgetInBytes = 1024 * 1024 * 1024; // 1GB
        // bufferPoolDesc->m_largestPooledAllocationSizeInBytes = (MaximumPointcloudCount+1) * sizeof (PointcloudAsset::CloudVertex);

        AZ::RPI::ResourcePoolAssetCreator creator;
        creator.Begin(AZ::Uuid::CreateRandom());
        creator.SetPoolDescriptor(AZStd::move(bufferPoolDesc));

        creator.SetPoolName("Pointclouds");
        creator.End(m_resourcePoolAsset);
        m_resourcePoolAsset.BlockUntilLoadComplete();
#endif
        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor Activated");
        const char* shaderFilePath = "Shaders/Pointclouds/Pointclouds.azshader";
        m_shader = AZ::RPI::LoadCriticalShader(shaderFilePath);

        if (!m_shader)
        {
            AZ_Error("PointcloudFeatureProcessor", false, "Failed to load required stars shader.");
            return;
        }
        AZ::Data::AssetBus::Handler::BusConnect(m_shader->GetAssetId());

        auto drawSrgLayout = m_shader->GetAsset()->GetDrawSrgLayout(m_shader->GetSupervariantIndex());
        AZ_Error(
            "PointcloudFeatureProcessor", drawSrgLayout, "Failed to get the draw shader resource group layout for the pointcloud shader.");
        if (drawSrgLayout)
        {
            m_drawSrg =
                AZ::RPI::ShaderResourceGroup::Create(m_shader->GetAsset(), m_shader->GetSupervariantIndex(), drawSrgLayout->GetName());
            m_pointSizeIndex.Reset();
            m_modelMatrixIndex.Reset();
        }

        m_drawListTag = m_shader->GetDrawListTag();

        auto viewportContextInterface = AZ::Interface<AZ::RPI::ViewportContextRequestsInterface>::Get();
        AZ_Assert(viewportContextInterface, "PointcloudFeatureProcessor requires the ViewportContextRequestsInterface.");
        auto viewportContext = viewportContextInterface->GetViewportContextByScene(GetParentScene());
        AZ_Assert(viewportContext, "PointcloudFeatureProcessor requires a valid ViewportContext.");
        m_viewportSize = viewportContext->GetViewportSize();

        EnableSceneNotification();

        AZ::RPI::ViewportContextIdNotificationBus::Handler::BusConnect(viewportContext->GetId());
    }

    PointcloudFeatureProcessorInterface::PointcloudHandle PointcloudFeatureProcessor::AquirePointcloud(const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData)
    {
        m_currentPointcloudDataIndex++;
        auto& pcData = m_pointcloudData[m_currentPointcloudDataIndex];
        pcData.m_index = m_currentPointcloudDataIndex;
        const uint32_t elementSize = sizeof(PointcloudAsset::CloudVertex);
        const uint32_t elementCount = static_cast<uint32_t>(cloudVertexData.size());
        const uint32_t bufferSize = elementCount * elementSize; // bytecount

        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor SetCloud %d, bytesize %d", elementCount, bufferSize);
        pcData.m_pointData = cloudVertexData;
        pcData.m_vertices = elementCount;

        AZ::RPI::CommonBufferDescriptor desc;
        desc.m_poolType = AZ::RPI::CommonBufferPoolType::ReadWrite;
        desc.m_bufferName = AZStd::string::format("PointcloudFeatureProcessor, %d", pcData.m_index);
        desc.m_byteCount = bufferSize;
        desc.m_elementSize = elementSize;
        desc.m_bufferData = pcData.m_pointData.data();

        AZStd::vector<uint8_t> bufferData;

        bufferData.resize_no_construct(desc.m_byteCount);
        memcpy(bufferData.data(), desc.m_bufferData, desc.m_byteCount);

        pcData.m_cloudVertexBuffer = AZ::RPI::BufferSystemInterface::Get()->CreateBufferFromCommonPool(desc);

        pcData.m_meshStreamBufferViews.front() =
            AZ::RHI::StreamBufferView(*pcData.m_cloudVertexBuffer->GetRHIBuffer(), 0, bufferSize, elementSize);
        UpdateDrawPacket();
        UpdateBackgroundClearColor();
        return pcData.m_index;
    }

    void PointcloudFeatureProcessor::UpdateDrawPacket()
    {
        for (auto& [_, pcData] : m_pointcloudData)
        {
            if (m_meshPipelineState && m_drawSrg && pcData.m_meshStreamBufferViews.front().GetByteCount() != 0)
            {
                pcData.m_drawPacket =
                    BuildDrawPacket(m_drawSrg, m_meshPipelineState, m_drawListTag, pcData.m_meshStreamBufferViews, pcData.m_vertices);
            }
        }
    }

    void PointcloudFeatureProcessor::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor OnAssetReloaded");
        UpdateDrawPacket();
        UpdateBackgroundClearColor();
    }

    void PointcloudFeatureProcessor::Deactivate()
    {
        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor Deactivated");
        AZ::Data::AssetBus::Handler::BusDisconnect(m_shader->GetAssetId());
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

    void PointcloudFeatureProcessor::UpdateBackgroundClearColor()
    {
        // This function is only necessary for now because the default clear value
        // color is not black, and is set in various .pass files in places a user
        // is unlikely to find.  Unfortunately, the viewport will revert to the
        // grey color when resizing momentarily.
        const AZ::RHI::ClearValue blackClearValue = AZ::RHI::ClearValue::CreateVector4Float(0.f, 0.f, 0.f, 0.f);
        AZ::RPI::PassFilter passFilter;
        AZStd::string slot;

        auto setClearValue = [&](AZ::RPI::Pass* pass) -> AZ::RPI::PassFilterExecutionFlow
        {
            AZ::Name slotName = AZ::Name::FromStringLiteral(slot, AZ::Interface<AZ::NameDictionary>::Get());
            if (auto binding = pass->FindAttachmentBinding(slotName))
            {
                binding->m_unifiedScopeDesc.m_loadStoreAction.m_clearValue = blackClearValue;
            }
            return AZ::RPI::PassFilterExecutionFlow::ContinueVisitingPasses;
        };

        slot = "SpecularOutput";
        passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ForwardPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);
        passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ForwardMSAAPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);

        slot = "ReflectionOutput";
        passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ReflectionGlobalFullscreenPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);
    }

    AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> PointcloudFeatureProcessor::BuildDrawPacket(
        const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup>& srg,
        const AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw>& pipelineState,
        const AZ::RHI::DrawListTag& drawListTag,
        const AZStd::span<const AZ::RHI::StreamBufferView>& streamBufferViews,
        uint32_t vertexCount)
    {
        AZ::RHI::DrawLinear drawLinear;
        drawLinear.m_vertexCount = vertexCount;
        drawLinear.m_vertexOffset = 0;
        drawLinear.m_instanceCount = 1;
        drawLinear.m_instanceOffset = 0;

        AZ::RHI::DrawPacketBuilder drawPacketBuilder;
        drawPacketBuilder.Begin(nullptr);
        drawPacketBuilder.SetDrawArguments(drawLinear);
        drawPacketBuilder.AddShaderResourceGroup(srg->GetRHIShaderResourceGroup());

        AZ::RHI::DrawPacketBuilder::DrawRequest drawRequest;
        drawRequest.m_listTag = drawListTag;
        drawRequest.m_pipelineState = pipelineState->GetRHIPipelineState();
        drawRequest.m_streamBufferViews = streamBufferViews;
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
                UpdateBackgroundClearColor();
            }
        }
        else if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::PassChanged)
        {
            if (m_meshPipelineState)
            {
                m_meshPipelineState->SetOutputFromScene(GetParentScene());
                m_meshPipelineState->Finalize();
                UpdateDrawPacket();
                UpdateBackgroundClearColor();
            }
        }
    }

    void PointcloudFeatureProcessor::UpdateShaderConstants()
    {
        for (auto& [_, pcData] : m_pointcloudData)
        {
            if (m_drawSrg && pcData.m_needSrgUpdate)
            {
                pcData.m_needSrgUpdate = false;
                AZ::Matrix4x4 orientation = AZ::Matrix4x4::CreateFromTransform(pcData.m_transform);
                m_drawSrg->SetConstant(m_modelMatrixIndex, orientation);
                m_drawSrg->SetConstant(m_pointSizeIndex, pcData.m_pointSize);
                m_drawSrg->Compile();
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
    }


} // namespace Pointcloud