### ImGui Provider Gem

The purpose of this Gem is to allow easy integration of GUI created with ImGui with O3DE components. Gem creates System Component which allows for registration of GUI features and manage displaying registered ones.

**Features and Integration:**

System component implements request bus, which allows to get and set active GUI. GUI is distinguished using `ImGuiFeaturePath`

```cpp
using ImGuiFeaturePath = AZ::IO::Path;

// names of consecutive tabs where feature should be located, e.g tools/LevelMonitoring/...
ImGuiFeaturePath pathToFeature;
```

Registered paths create tree-like structure, so it is possible to create multiple depth levels within tabs.

ImGui Feature needs to be within Component with Handler for ImGuiProviderNotificationBus. Method drawing ImGui should be implemented as override of OnImGuiUpdate method. Paths to registered features must be unique within project.

Below example on how to register new feature during component activation:

```cpp
void ExampleComponent::Activate()
{
    /*
    some implementation
    */
    auto pathToFeature = ImGuiProvider::ImGuiFeaturePath{ "Tools/ExampleFeature" };
    ImGuiProvider::ImGuiProviderNotificationBus::Handler::BusConnect(pathToFeature);
    /*
    some implementation
    */
}
```

Registered features are present as long as the lifetime of the handler which defining it. ImGuiProviderSystemComponent constantly monitors registered handlers and adjust displayed gui if needed.

Gem handles correct displaying of debug menu available using `home` button. If Gem detects debug menu, registered GUIs disappear. After disabling debug menu, previous state of registered features is restored.
If Gem detects custom GUI feature registered in Editor, viewport icons are moved to prevent covering the registered features.
