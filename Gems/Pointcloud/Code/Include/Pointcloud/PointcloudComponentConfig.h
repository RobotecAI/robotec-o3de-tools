/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "PointcloudAsset.h"
#include "PointcloudFeatureProcessorInterface.h"
#include "PointcloudTypeIds.h"

namespace Pointcloud
{
    //! Configuration for a Pointcloud component.
    class PointcloudComponentConfig final : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(PointcloudComponentConfig, PointcloudComponentConfigTypeId);

        PointcloudComponentConfig() = default;
        ~PointcloudComponentConfig() = default;

        static void Reflect(AZ::ReflectContext* context);

        //! Owning entity identifier. This field is set by the PointcloudEditorComponent component during its activation.
        AZ::EntityId m_editorEntityId;

        //! Size of rendered points.
        float m_pointSize = 1.0f;

        //! Runtime handle to the point cloud resource managed by the feature processor.
        PointcloudFeatureProcessorInterface::PointcloudHandle m_pointcloudHandle =
            PointcloudFeatureProcessorInterface::InvalidPointcloudHandle;

        //! Asset reference to the point cloud data to be loaded and rendered.
        AZ::Data::Asset<PointcloudAsset> m_pointcloudAsset = {};
    };
} // namespace Pointcloud
