
#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzQtComponents/Utilities/QtPluginPaths.h>
#include <AzTest/AzTest.h>
#include <AzTest/GemTestEnvironment.h>
#include <UnitTest/ToolsTestApplication.h>

#include <QApplication>
#include <gtest/gtest.h>

namespace UnitTest
{

    class ImGuiProviderTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        ImGuiProviderTestEnvironment() = default;
        ~ImGuiProviderTestEnvironment() override = default;
    };

    void ImGuiProviderTestEnvironment::AddGemsAndComponents()
    {
        // We dynamically load the PhysX, LmbrCentral gems.
        // After loading the Gem it creates all its internal components.
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "ImGuiProvider" }));
    }

    AZ::ComponentApplication* ImGuiProviderTestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("ImGuiProviderTestEnvironment");
    }

    void ImGuiProviderTestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }

    // Any critical failures in the tests will be converted to exceptions and propagated to the test runner.
    class ThrowListener : public testing::EmptyTestEventListener
    {
        void OnTestPartResult(const testing::TestPartResult& result) override
        {
            if (result.type() == testing::TestPartResult::kFatalFailure)
            {
                throw ::testing::AssertionException(result);
            }
        }
    };

} // namespace UnitTest

// required to support running tests with PhysX.Editor.Gem
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::ImGuiProviderTestEnvironment() });
    testing::UnitTest::GetInstance()->listeners().Append(new UnitTest::ThrowListener);
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
