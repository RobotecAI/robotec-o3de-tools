#pragma once

#include <AzCore/std/string/string.h>
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
    AZ_TYPE_INFO_SPECIALIZE(TerrainShaperActions, "{EBAA0F99-102F-4EAB-AFB4-0A5086E679A2}");

    enum class TerrainShaperBrushTypes
    {
        Circle = 0,
        Rectangle = 1,
        Square = 2,
        Triangle = 3
    };
    AZ_TYPE_INFO_SPECIALIZE(TerrainShaperBrushTypes, "{5704010D-EAA7-4D28-A84B-BC448433106F}");

    struct BrushInfo
    {
        TerrainShaperBrushTypes m_brushType;
        AZStd::string m_iconPath;  // Icon path in .qrc
        // AZStd::string toolTip;  // Tooltip text
    };
    AZ_TYPE_INFO_SPECIALIZE(BrushInfo, "{638C66D3-096B-4476-9021-D4EF1616498F}");

    // class TerrainShaperConfig
    // {
    //
    // };

} // TerrainShaper::Config
