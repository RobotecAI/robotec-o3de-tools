#include "Clients/Fixtures/MenuTreeFixture.hpp"
#include <ImGuiProvider/ImGuiProviderBus.h>
#include <gtest/gtest.h>

namespace UnitTest
{
    TEST_F(MenuTreeFixture, TestLabellAccess)
    {
        root.SetNodeLabel(ImGuiProvider::ImGuiFeaturePath("Test/Label"));
        ASSERT_STREQ(root.GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("Test/Label").c_str());
    }
} // namespace UnitTest
