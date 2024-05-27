/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/base.h>
#include <Atom/RPI.Public/FeatureProcessor.h>

namespace Pointcloud
{
    class Pointcloud;

    using PointcloudHandle = AZStd::shared_ptr<Pointcloud>;

    // PointcloudFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
    class PointcloudFeatureProcessorInterface
        : public AZ::RPI::FeatureProcessor
    {
    public:
        AZ_RTTI(PointcloudFeatureProcessorInterface, "{B44ECAE2-F2E3-4C9D-BB6C-0C5E3ACF0FD6}", AZ::RPI::FeatureProcessor);

    };
}
