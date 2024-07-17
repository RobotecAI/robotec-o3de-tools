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
#include <AzCore/Component/Component.h>
#include <AzCore/Math/Transform.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Pointcloud/PointcloudAsset.h>
namespace Pointcloud
{

    using PointcloudHandle = int32_t;
    constexpr PointcloudHandle InvalidPointcloudHandle = -1;

    // PointcloudFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
    class PointcloudFeatureProcessorInterface
        : public AZ::RPI::FeatureProcessor
    {
    public:

        AZ_RTTI(PointcloudFeatureProcessorInterface, "{8597AF27-EB4E-4363-8889-3BFC2AF5D2EC}", AZ::RPI::FeatureProcessor);

        virtual void SetTransform(const PointcloudHandle& handle, const AZ::Transform &transform)= 0;
        virtual void SetPointSize(const PointcloudHandle& handle, float pointSize)= 0;
        virtual PointcloudHandle AquirePointcloud(const AZ::Data::Asset<PointcloudAsset> &asset )= 0;

    };
}
