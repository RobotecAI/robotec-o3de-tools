#include "Clients/Fixtures/MenuTreeFixture.hpp"
#include <gtest/gtest.h>

namespace UnitTest
{
    TEST_F(MenuTreeFixture, TestAddingNode)
    {
        root.AddChild("test");
        root.AddChild("test2");
        EXPECT_TRUE(root.HasChildren());
        EXPECT_TRUE(root.HasChild("test"));
        EXPECT_TRUE(root.HasChild("test2"));
    }

    TEST_F(MenuTreeFixture, TestRemovingNode)
    {
        root.AddChild("test");
        root.AddChild("test2");
        EXPECT_TRUE(root.HasChildren());
        root.RemoveChild("test");
        root.RemoveChild("test2");
        EXPECT_FALSE(root.HasChildren());
    }
} // namespace UnitTest
