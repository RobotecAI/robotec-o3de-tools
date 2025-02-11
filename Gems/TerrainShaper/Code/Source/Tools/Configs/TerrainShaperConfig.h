#pragma once

#include <AzCore/RTTI/TypeInfo.h>

namespace TerrainShaper::Config
{
    enum class TerrainShaperActions
    {
        Flatten = 0,
        Raise   = 1,
        Lower   = 2,
        Smooth  = 3
    };

    AZ_TYPE_INFO_SPECIALIZE(TerrainShaperActions, "{D634499B-42A8-4C3E-9DCA-497F5214D6F5}");

    class TerrainShaperConfig
    {

    };

} // TerrainShaper::Config
