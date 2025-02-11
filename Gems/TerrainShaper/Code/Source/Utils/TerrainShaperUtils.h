#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>

namespace TerrainShaper::Utils
{
    AZStd::vector<AZ::EntityId> GetAllTerrains();
    AZStd::vector<AZ::Vector3> GetTerrainGradient(const AZ::EntityId& terrainId, const AZ::Vector3& position);

} // TerrainShaper::Utils
