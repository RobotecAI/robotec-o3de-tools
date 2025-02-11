#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>

namespace TerrainShaper::Utils
{
    AZStd::vector<AZ::EntityId> GetAllTerrains();

} // TerrainShaper::Utils
