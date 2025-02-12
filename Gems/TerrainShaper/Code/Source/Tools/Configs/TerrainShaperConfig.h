#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/RTTI/TypeInfo.h>

namespace TerrainShaper::Config
{
    enum class ShaperActions
    {
        Flatten = 0,
        Raise   = 1,
        Lower   = 2,
        Smooth  = 3
    };
    AZ_TYPE_INFO_SPECIALIZE(ShaperActions, "{EBAA0F99-102F-4EAB-AFB4-0A5086E679A2}");

    enum class BrushTypes
    {
        Circle = 0,
        Rectangle = 1,
        Square = 2,
        Triangle = 3
    };
    AZ_TYPE_INFO_SPECIALIZE(BrushTypes, "{5704010D-EAA7-4D28-A84B-BC448433106F}");

    struct BrushInfo
    {
        BrushTypes m_brushType;
        AZStd::string m_iconPath;  // Icon path in .qrc
        // AZStd::string toolTip;  // Tooltip text
    };
    AZ_TYPE_INFO_SPECIALIZE(BrushInfo, "{638C66D3-096B-4476-9021-D4EF1616498F}");

    struct TerrainSelectSettings
    {
        bool m_enableOutline;
        bool m_enableFocus;
    };
    AZ_TYPE_INFO_SPECIALIZE(TerrainSelectSettings, "{532CBC23-CD74-4631-94D8-909635943EC5}");

    // class TerrainShaperConfig
    // {
    //
    // };

} // TerrainShaper::Config
