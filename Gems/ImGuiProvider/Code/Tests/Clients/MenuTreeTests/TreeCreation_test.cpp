#include "Clients/Fixtures/MenuTreeFixture.hpp"
#include "Utils/MenuTreeUtilities.h"
#include <ImGuiProvider/ImGuiProviderBus.h>
#include <gtest/gtest.h>

namespace UnitTest
{

    int GetNumberOfNodesToLeaves(const ImGuiProvider::GUIMenuNode& root)
    {
        using namespace ImGuiProvider;
        int counter = 0;
        // if no children assigned, assigned 0 is correct
        for (const auto& [name, child] : root.GetAllChildren())
        {
            if (child.HasChildren())
            {
                counter += GetNumberOfNodesToLeaves(child);
            }

            counter++;
        }
        return counter;
    }

    TEST_F(MenuTreeFixture, TestCreateTreeOneLevel)
    {
        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test"));

        auto nodesToTest = root.IterateParents(ImGuiProvider::ImGuiFeaturePath("Test"));
        EXPECT_EQ(nodesToTest.size(), 1);
        ASSERT_NE(nodesToTest.front().first, nullptr);
        EXPECT_EQ(nodesToTest.front().second, "Test");
        ASSERT_STREQ(nodesToTest.front().first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("Test").c_str());
    }

    TEST_F(MenuTreeFixture, TestTreeCreate2LvlStructure)
    {
        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test2"));
        EXPECT_EQ(GetNumberOfNodesToLeaves(root), 2);

        auto nodesToTest = root.IterateParents(ImGuiProvider::ImGuiFeaturePath("Test/Test2"));
        EXPECT_EQ(nodesToTest.size(), 2);
        ASSERT_NE(nodesToTest.front().first, nullptr);
        EXPECT_EQ(nodesToTest.front().second, "Test");
        ASSERT_STREQ(nodesToTest.front().first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("Test").c_str());
        ASSERT_NE(nodesToTest.at(1).first, nullptr);
        EXPECT_EQ(nodesToTest.at(1).second, "Test2");
        ASSERT_STREQ(nodesToTest.at(1).first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("Test/Test2").c_str());
    }
    TEST_F(MenuTreeFixture, TestTreeCreateMultiBranchStructure)
    {
        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test2/Test3"));
        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test4/Test5"));
        EXPECT_EQ(GetNumberOfNodesToLeaves(root), 5);
        {
            auto nodesToTest = root.IterateParents(ImGuiProvider::ImGuiFeaturePath("Test/Test2/Test3"));
            EXPECT_EQ(nodesToTest.size(), 3);
            ASSERT_NE(nodesToTest.front().first, nullptr);
            EXPECT_EQ(nodesToTest.front().second, "Test");
            ASSERT_STREQ(nodesToTest.front().first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("Test").c_str());
            ASSERT_NE(nodesToTest.at(1).first, nullptr);
            EXPECT_EQ(nodesToTest.at(1).second, "Test2");
            ASSERT_STREQ(nodesToTest.at(1).first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("Test/Test2").c_str());
            ASSERT_NE(nodesToTest.at(2).first, nullptr);
            EXPECT_EQ(nodesToTest.at(2).second, "Test3");
            ASSERT_STREQ(nodesToTest.at(2).first->GetNodeLabel().c_str(), ImGuiProvider::ImGuiFeaturePath("Test/Test2/Test3").c_str());
        }
        {
            auto nodesToTest = root.IterateParents(ImGuiProvider::ImGuiFeaturePath("Test/Test4/Test5"));
            EXPECT_EQ(nodesToTest.size(), 3);
            ASSERT_NE(nodesToTest.front().first, nullptr);
            EXPECT_EQ(nodesToTest.front().second, "Test");
            EXPECT_EQ(nodesToTest.front().first->GetNodeLabel(), ImGuiProvider::ImGuiFeaturePath("Test"));
            ASSERT_NE(nodesToTest.at(1).first, nullptr);
            EXPECT_EQ(nodesToTest.at(1).second, "Test4");
            EXPECT_EQ(nodesToTest.at(1).first->GetNodeLabel(), ImGuiProvider::ImGuiFeaturePath("Test/Test4"));
            ASSERT_NE(nodesToTest.at(2).first, nullptr);
            EXPECT_EQ(nodesToTest.at(2).second, "Test5");
            EXPECT_EQ(nodesToTest.at(2).first->GetNodeLabel(), ImGuiProvider::ImGuiFeaturePath("Test/Test4/Test5"));
        }
    }

    TEST_F(MenuTreeFixture, TestTreeRemoval)
    {
        // one level deeper insertion
        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test2/Test3"));
        // insertion of deeper node without created previous node
        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test4/Test5"));
        // create sub branch at level test4
        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test4/Test6/Test7"));

        ImGuiProvider::InsertIntoTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test8/Test9"));

        // expected tree
        /*
        test
            test2
                test3
            test4
                test5
                test6
                    test7
            test8
                test9u
        */

        EXPECT_EQ(GetNumberOfNodesToLeaves(root), 9);

        // remove whole branch
        ImGuiProvider::RemoveFromTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test2/Test3"));
        EXPECT_FALSE(root.HasChild("Test2"));
        ASSERT_EQ(root.GetChildPtr("Test2"), nullptr);
        EXPECT_EQ(GetNumberOfNodesToLeaves(root), 7);

        // remove half of the branch - remove missed nodes
        ImGuiProvider::RemoveFromTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test8"));
        EXPECT_FALSE(root.HasChild("Test8"));
        ASSERT_EQ(root.GetChildPtr("Test8"), nullptr);
        EXPECT_EQ(GetNumberOfNodesToLeaves(root), 5);

        // remove node holding sub-branches -> Remove sub-branches as well
        ImGuiProvider::RemoveFromTree(root, ImGuiProvider::ImGuiFeaturePath("Test/Test4"));
        EXPECT_FALSE(root.HasChild("Test4"));
        ASSERT_EQ(root.GetChildPtr("Test4"), nullptr);
        EXPECT_EQ(GetNumberOfNodesToLeaves(root), 1);
    }

} // namespace UnitTest
