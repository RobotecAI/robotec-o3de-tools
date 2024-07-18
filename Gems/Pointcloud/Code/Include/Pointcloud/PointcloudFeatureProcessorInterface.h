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

   using PointcloudHandle = AZStd::shared_ptr<Pointcloud>;

   // PointcloudFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
   class PointcloudFeatureProcessorInterface
       : public AZ::RPI::FeatureProcessor
   {
   public:
       AZ_RTTI(PointcloudFeatureProcessorInterface, "{8597AF27-EB4E-4363-8889-3BFC2AF5D2EC}", AZ::RPI::FeatureProcessor);
//       struct CloudVertex
//       {
//           AZStd::array<float, 3> m_position;
//           uint32_t m_color;
//       };

       virtual void SetTransform(const AZ::Transform &transform)= 0;
       virtual void SetPointSize(float pointSize)= 0;
       virtual void SetCloud(const AZStd::vector<PointcloudAsset::CloudVertex>& cloudVertexData)= 0;

   };
}