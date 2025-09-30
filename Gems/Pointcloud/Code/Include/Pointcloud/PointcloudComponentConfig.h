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

namespace Pointcloud
{
    class PointcloudComponentConfig final : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(PointcloudComponentConfig, );

        PointcloudComponentConfig() = default;
        ~PointcloudComponentConfig() = default;

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_editorEntityId;
        float m_pointSize = 1.0f;
        PointcloudFeatureProcessorInterface::PointcloudHandle m_pointcloudHandle =
            PointcloudFeatureProcessorInterface::InvalidPointcloudHandle;
        AZ::Data::Asset<PointcloudAsset> m_pointcloudAsset = {};
    };
} // namespace Pointcloud
