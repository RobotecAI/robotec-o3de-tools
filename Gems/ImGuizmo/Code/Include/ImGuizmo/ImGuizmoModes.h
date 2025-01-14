// https://github.com/CedricGuillemet/ImGuizmo
// v1.91.3 WIP
//
// The MIT License(MIT)
//
// Copyright(c) 2021 Cedric Guillemet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// -------------------------------------------------------------------------------------------
// History :
// 2025/01/10 Adjustments in library during integration into O3DE (moved enums to ImGuizmoModes.h)

#pragma once
namespace ImGuizmo
{
    enum OPERATION
    {
        TRANSLATE_X = (1u << 0),
        TRANSLATE_Y = (1u << 1),
        TRANSLATE_Z = (1u << 2),
        ROTATE_X = (1u << 3),
        ROTATE_Y = (1u << 4),
        ROTATE_Z = (1u << 5),
        ROTATE_SCREEN = (1u << 6),
        SCALE_X = (1u << 7),
        SCALE_Y = (1u << 8),
        SCALE_Z = (1u << 9),
        BOUNDS = (1u << 10),
        SCALE_XU = (1u << 11),
        SCALE_YU = (1u << 12),
        SCALE_ZU = (1u << 13),

        TRANSLATE = TRANSLATE_X | TRANSLATE_Y | TRANSLATE_Z,
        ROTATE = ROTATE_X | ROTATE_Y | ROTATE_Z | ROTATE_SCREEN,
        SCALE = SCALE_X | SCALE_Y | SCALE_Z,
        SCALEU = SCALE_XU | SCALE_YU | SCALE_ZU, // universal
        UNIVERSAL = TRANSLATE | ROTATE | SCALEU
    };

    inline OPERATION operator|(OPERATION lhs, OPERATION rhs)
    {
        return static_cast<OPERATION>(static_cast<int>(lhs) | static_cast<int>(rhs));
    }

    enum MODE : unsigned int
    {
        LOCAL,
        WORLD
    };
} // namespace ImGuizmo