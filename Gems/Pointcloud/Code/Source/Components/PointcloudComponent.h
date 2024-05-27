/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Components/PointcloudComponentController.h>
#include <AzFramework/Components/ComponentAdapter.h>

namespace Pointcloud
{
    inline constexpr AZ::TypeId PointcloudComponentTypeId { "{9F9534D4-A840-4657-9B02-1102A2B9CA3E}" };

    class PointcloudComponent final
        : public AzFramework::Components::ComponentAdapter<PointcloudComponentController, PointcloudComponentConfig>
    {
    public:
        using BaseClass = AzFramework::Components::ComponentAdapter<PointcloudComponentController, PointcloudComponentConfig>;
        AZ_COMPONENT(PointcloudComponent, PointcloudComponentTypeId, BaseClass);

        PointcloudComponent() = default;
        PointcloudComponent(const PointcloudComponentConfig& config);

        static void Reflect(AZ::ReflectContext* context);
    };
}
