#include "MenuTreeUtilities.h"
#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>

namespace ImGuiProvider
{
    void InsertIntoTree(GUIMenuNode& root, const ImGuiFeaturePath& path)
    {
        GUIMenuNode* current = &root; // Start at the root node
        ImGuiFeaturePath nodesLabel(""); // each label points to absolute path from root to the node
        for (const auto& level : path)
        {
            nodesLabel = nodesLabel / level;
            if (!current->HasChild(level.String()))
            {
                current->AddChild(level.String()); // Create new node explicitly
            }
            current = current->GetChildPtr(level.String()); // Move to the child node
            current->SetNodeLabel(nodesLabel);
        }
    }

    void RemoveFromTree(GUIMenuNode& root, const ImGuiFeaturePath& path)
    {
        GUIMenuNode* current = &root; // Start at the root node
        auto parents = current->IterateParents(path);
        AZStd::optional<AZStd::string> emptyNodeToRemove;
        auto removeNodes = [&emptyNodeToRemove](const NodeWithName& elem)
        {
            if (!elem.first->HasChildren())
            {
                emptyNodeToRemove = elem.second;
            }
            else
            {
                // queue data for next iteration
                if (emptyNodeToRemove.has_value())
                {
                    elem.first->RemoveChild(emptyNodeToRemove.value());
                    emptyNodeToRemove = AZStd::nullopt;
                }
                // after removal, update state of children
                if (!elem.first->HasChildren())
                {
                    emptyNodeToRemove = elem.second;
                }
            }
        };

        if (parents.back().first->HasChildren())
        {
            AZ_Warning("ImGuiProvider::MenuTreeTools", false, "Requested removal of nod with children, children will be removed");
            auto paths = parents.back().first->IterateChildren(true);
            AZStd::for_each(
                paths.rbegin(),
                paths.rend(),
                [&root](const NodeWithName& elem)
                {
                    RemoveFromTree(root, elem.first->GetNodeLabel());
                });
        }
        else
        {
            AZStd::for_each(parents.rbegin(), parents.rend(), removeNodes);
        }
    }

    const ImGuiFeaturePath GUIMenuNode::GetNodeLabel() const
    {
        return m_nodeLabel;
    }

    void GUIMenuNode::SetNodeLabel(const ImGuiFeaturePath& label)
    {
        m_nodeLabel = label;
    }
    void GUIMenuNode::AddChild(const AZStd::string& childName)
    {
        m_children[childName] = GUIMenuNode();
    }

    void GUIMenuNode::RemoveChild(const AZStd::string& childName)
    {
        m_children.erase(childName);
    }

    AZStd::vector<NodeWithName> GUIMenuNode::IterateParents(const ImGuiFeaturePath& path)
    {
        AZStd::vector<NodeWithName> parentNodes;
        GUIMenuNode* current = this; // Start at the object where method was called
        for (const auto& level : path)
        {
            if (current->HasChild(level.String()))
            {
                current = current->GetChildPtr(level.String()); // Move to the child node
            }
            parentNodes.push_back(AZStd::make_pair(current, level));
        }
        return parentNodes;
    }

    AZStd::vector<NodeWithName> GUIMenuNode::IterateChildren(bool leavesOnly)
    {
        AZStd::vector<NodeWithName> childrenNodes;
        for (auto& [name, child] : m_children)
        {
            if (child.HasChildren())
            {
                auto tempPath = child.IterateChildren(leavesOnly);
                childrenNodes.insert(childrenNodes.end(), tempPath.begin(), tempPath.end());
            }
            // add either all or only leaf node
            if (!leavesOnly)
            {
                NodeWithName node(&child, name);
                childrenNodes.push_back(node);
            }
            else if (!child.HasChildren())
            {
                NodeWithName node(&child, name);
                childrenNodes.push_back(node);
            }
        }
        return childrenNodes;
    }

    const AZStd::unordered_map<AZStd::string, GUIMenuNode> GUIMenuNode::GetAllChildren() const
    {
        return m_children;
    }

    bool GUIMenuNode::HasChild(const AZStd::string& childName) const
    {
        return m_children.contains(childName);
    }

    bool GUIMenuNode::HasChildren() const
    {
        return !m_children.empty();
    }

    GUIMenuNode* GUIMenuNode::GetChildPtr(const AZStd::string& childName)
    {
        if (!HasChild(childName))
        {
            return nullptr;
        }
        return &m_children[childName];
    }

} // namespace ImGuiProvider
