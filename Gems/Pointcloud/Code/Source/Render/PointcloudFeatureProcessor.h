/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Pointcloud/PointcloudFeatureProcessorInterface.h>

namespace Pointcloud
{
    class PointcloudFeatureProcessor final
        : public PointcloudFeatureProcessorInterface
    {
    public:
        AZ_RTTI(PointcloudFeatureProcessor, "{CD4A7046-EC2E-424B-A27B-381DFF98AF9A}", PointcloudFeatureProcessorInterface);
        AZ_CLASS_ALLOCATOR(PointcloudFeatureProcessor, AZ::SystemAllocator)

        static void Reflect(AZ::ReflectContext* context);

        PointcloudFeatureProcessor() = default;
        virtual ~PointcloudFeatureProcessor() = default;

        // FeatureProcessor overrides
        void Activate() override;
        void Deactivate() override;
        void Simulate(const FeatureProcessor::SimulatePacket& packet) override;

    };
}
