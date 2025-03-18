#pragma once

#include "Utils/MenuTreeUtilities.h"
#include <gtest/gtest.h>

namespace UnitTest
{

    class MenuTreeFixture : public ::testing::Test
    {
    public:
        void SetUp() override
        {
        }

        void TearDown() override
        {
        }

    protected:
        // only needed root defined as protected field of this structure
        ImGuiProvider::GUIMenuNode root;
    };

} // namespace UnitTest
