#pragma once
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <ImGuiProvider/ImGuiProviderBus.h>

namespace ImGuiProvider
{
    // structure of menu is tree-like to allow multiple level of features, depends on registered features
    class GUIMenuNode;
    using NodeWithName = AZStd::pair<GUIMenuNode*, AZStd::string>;

    class GUIMenuNode
    {
    public:
        const ImGuiFeaturePath GetNodeLabel() const;
        void SetNodeLabel(const ImGuiFeaturePath& label);
        void AddChild(const AZStd::string& childName);
        void RemoveChild(const AZStd::string& childName);
        // returns node from root to node pointed by path
        AZStd::vector<NodeWithName> IterateParents(const ImGuiFeaturePath& path);
        // returns every node from root node to the leaves. If leaves onl, intermediate nodes are skipped
        AZStd::vector<NodeWithName> IterateChildren(bool leavesOnly = false);
        // returns number of children of `this`. Doesn't include deeper nodes
        const AZStd::unordered_map<AZStd::string, GUIMenuNode> GetAllChildren() const;
        bool HasChild(const AZStd::string& childName) const;
        bool HasChildren() const;
        GUIMenuNode* GetChildPtr(const AZStd::string& childName);

    private:
        AZStd::unordered_map<AZStd::string, GUIMenuNode> m_children;
        // node label is assigned during InsertIntoTree, it is path from the root to the node
        ImGuiFeaturePath m_nodeLabel = "";
    };

    void InsertIntoTree(GUIMenuNode& root, const ImGuiFeaturePath& path);
    // method meant to remove tree leaves,
    void RemoveFromTree(GUIMenuNode& root, const ImGuiFeaturePath& path);

} // namespace ImGuiProvider
