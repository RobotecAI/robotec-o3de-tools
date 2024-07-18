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
#include <Pointcloud/PointcloudAsset.h>
namespace Pointcloud
{
   class Pointcloud;

   // PointcloudFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
   class PointcloudFeatureProcessorInterface
       : public AZ::RPI::FeatureProcessor
   {
   public:
       using PointcloudHandle = int;
       constexpr static PointcloudHandle InvalidPointcloudHandle = -1;
       AZ_RTTI(PointcloudFeatureProcessorInterface, "{8597AF27-EB4E-4363-8889-3BFC2AF5D2EC}", AZ::RPI::FeatureProcessor);

       virtual void SetTransform(const PointcloudHandle& handle, const AZ::Transform &transform)= 0;
       virtual void SetPointSize(const PointcloudHandle& handle, float pointSize)= 0;
       virtual PointcloudHandle AquirePointcloud(const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData)= 0;
       virtual void SetVisibility(const PointcloudHandle& handle, bool visible)= 0;
       virtual void ReleasePointcloud(const PointcloudHandle& handle)= 0;

   };
}