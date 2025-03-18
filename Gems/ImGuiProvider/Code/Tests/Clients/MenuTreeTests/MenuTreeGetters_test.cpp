#include "Clients/Fixtures/MenuTreeFixture.hpp"
#include <ImGuiProvider/ImGuiProviderBus.h>
#include <gtest/gtest.h>

namespace UnitTest
{
    // in these tests trees are created via already tested primitive methods
    TEST_F(MenuTreeFixture, TestGettingChildPtr)
    {
        root.AddChild("test");
        root.AddChild("test2");
        EXPECT_TRUE(root.HasChildren());
        EXPECT_TRUE(root.HasChild("test"));
        EXPECT_TRUE(root.HasChild("test2"));
        auto child = root.GetChildPtr("test");
        EXPECT_NE(child, nullptr);
        auto child2 = root.GetChildPtr("test2");
        EXPECT_NE(child2, nullptr);
    }

    TEST_F(MenuTreeFixture, TestGettingAllChildren)
    {
        // setup
        root.AddChild("test");
        root.AddChild("test2");
        root.AddChild("test3");

        auto allChildren = root.GetAllChildren();
        EXPECT_EQ(allChildren.size(), 3);
        // test returned values;
        EXPECT_TRUE(allChildren.contains("test"));
        ASSERT_NE(&allChildren["test"], nullptr);
        EXPECT_TRUE(allChildren.contains("test2"));
        ASSERT_NE(&allChildren["test2"], nullptr);
        EXPECT_TRUE(allChildren.contains("test3"));
        ASSERT_NE(&allChildren["test3"], nullptr);
    }

    TEST_F(MenuTreeFixture, TestIterateParents)
    {
        // setup using already tested "atomic" functions
        root.AddChild("test");
        root.AddChild("test2");
        auto child = root.GetChildPtr("test");
        ASSERT_NE(child, nullptr);
        child->SetNodeLabel("test");
        auto child2 = root.GetChildPtr("test2");
        ASSERT_NE(child2, nullptr);
        child2->SetNodeLabel("test2");
        child2->AddChild("test3");
        auto childDeeperLevel = child2->GetChildPtr("test3");
        ASSERT_NE(childDeeperLevel, nullptr);
        childDeeperLevel->SetNodeLabel("test2/test3");
        {
            auto nodesToTest = root.IterateParents(ImGuiProvider::ImGuiFeaturePath("test"));
            EXPECT_EQ(nodesToTest.size(), 1);
            ASSERT_NE(&nodesToTest.front().first, nullptr);
            EXPECT_EQ(nodesToTest.front().second, "test");
            ASSERT_STREQ(nodesToTest.front().first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("test").c_str());
        }
        {
            auto nodesToTest = root.IterateParents(ImGuiProvider::ImGuiFeaturePath("test2/test3"));
            EXPECT_EQ(nodesToTest.size(), 2);
            ASSERT_NE(&nodesToTest.front().first, nullptr);
            EXPECT_EQ(nodesToTest.front().second, "test2");
            ASSERT_STREQ(nodesToTest.front().first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("test2").c_str());
            ASSERT_NE(&nodesToTest.at(1).first, nullptr);
            EXPECT_EQ(nodesToTest.at(1).second, "test3");
            ASSERT_STREQ(nodesToTest.at(1).first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("test2/test3").c_str());
        }
    }

    TEST_F(MenuTreeFixture, TestIterateChildren)
    {
        // setup using already tested "atomic" functions
        root.AddChild("test");
        root.AddChild("test2");

        auto child = root.GetChildPtr("test");
        ASSERT_NE(child, nullptr);
        child->SetNodeLabel("test");

        auto child2 = root.GetChildPtr("test2");
        ASSERT_NE(child2, nullptr);
        child2->SetNodeLabel("test2");

        child2->AddChild("test3");
        child2->AddChild("test4");

        auto child3 = child2->GetChildPtr("test3");
        ASSERT_NE(child3, nullptr);
        child3->SetNodeLabel("test2/test3");

        auto child4 = child2->GetChildPtr("test4");
        ASSERT_NE(child4, nullptr);
        child4->SetNodeLabel("test2/test4");

        child4->AddChild("test5");

        auto child5 = child4->GetChildPtr("test5");
        ASSERT_NE(child5, nullptr);
        child5->SetNodeLabel("test2/test4/test5");
        // expected structure
        /*
            test
            test2
                test3
                test4
                    test5
        */

        // order is not guaranteed, wont be tested
        // get all children
        {
            auto nodesToTest = root.IterateChildren();
            EXPECT_EQ(nodesToTest.size(), 5);
            for (const auto& node : nodesToTest)
            {
                ASSERT_NE(node.first, nullptr);
            }
        }
        // get only leaves
        {
            auto nodesToTest = root.IterateChildren(true);
            EXPECT_EQ(nodesToTest.size(), 3);
            for (const auto& node : nodesToTest)
            {
                ASSERT_NE(node.first, nullptr);
            }
        }
    }
} // namespace UnitTest
