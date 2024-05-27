/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudFeatureProcessor.h"

namespace Pointcloud
{
    void PointcloudFeatureProcessor::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext
                ->Class<PointcloudFeatureProcessor, FeatureProcessor>()
                ;
        }
    }

    void PointcloudFeatureProcessor::Activate()
    {

    }

    void PointcloudFeatureProcessor::Deactivate()
    {
        
    }

    void PointcloudFeatureProcessor::Simulate([[maybe_unused]] const FeatureProcessor::SimulatePacket& packet)
    {
        
    }    
}
