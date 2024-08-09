/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/FeatureProcessor.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/base.h>
#include <Pointcloud/PointcloudAsset.h>
namespace Pointcloud
{
    class Pointcloud;

    // PointcloudFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
    class PointcloudFeatureProcessorInterface : public AZ::RPI::FeatureProcessor
    {
    public:
        using PointcloudHandle = int;
        constexpr static PointcloudHandle InvalidPointcloudHandle = -1;
        AZ_RTTI(PointcloudFeatureProcessorInterface, "{8597AF27-EB4E-4363-8889-3BFC2AF5D2EC}", AZ::RPI::FeatureProcessor);

        //! Set the transform of a pointcloud
        //! @param handle The handle of the pointcloud obtained from AcquirePointcloud
        virtual void SetTransform(const PointcloudHandle& handle, const AZ::Transform& transform) = 0;

        //! Set the point size of a pointcloud
        //! @param handle The handle of the pointcloud obtained from AcquirePointcloud
        virtual void SetPointSize(const PointcloudHandle& handle, float pointSize) = 0;

        //! Allocate resources and return a handle to the pointcloud
        //! @param cloudVertexData The vertex data of the pointcloud
        virtual PointcloudHandle AcquirePointcloud(const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData) = 0;

        //! Allocate resources and return a handle to the pointcloud
        //! @param pointcloudAsset The asset of the pointcloud
        virtual PointcloudHandle AcquirePointcloudFromAsset(AZ::Data::Asset<PointcloudAsset> pointcloudAsset) = 0;

        //! Update the vertex data of a pointcloud, resizes the pointcloud if necessary
        //! @param handle The handle of the pointcloud obtained from AcquirePointcloud
        //! @param cloudVertexData The vertex data of the pointcloud
        //! @param startIdx The start index of the vertex data to update (default 0)
        virtual void UpdatePointCloud(
            PointcloudHandle PointcloudDataIndex,
            const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData,
            size_t startIdx = 0) = 0;

        //! Set the visibility of a pointcloud
        //! @param handle The handle of the pointcloud obtained from AcquirePointcloud
        virtual void SetVisibility(const PointcloudHandle& handle, bool visible) = 0;

        //! Release the resources of a pointcloud
        //! @param handle The handle of the pointcloud obtained from AcquirePointcloud
        virtual void ReleasePointcloud(const PointcloudHandle& handle) = 0;
    };
} // namespace Pointcloud