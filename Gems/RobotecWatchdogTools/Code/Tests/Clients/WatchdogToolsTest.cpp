
/*
 * Copyright (c) 2024 Robotec.ai
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AzCore/Component/Entity.h"
#include "AzCore/Module/ModuleManagerBus.h"
#include "AzCore/Settings/SettingsRegistryImpl.h"
#include "AzCore/std/smart_ptr/make_shared.h"
#include "Clients/WatchdogToolsSettings.h"
#include "Clients/WatchdogToolsSystemComponent.h"

#include <AzCore/Module/DynamicModuleHandle.h>
#include <AzCore/Module/Environment.h>
#include <AzCore/Module/Module.h>
#include <AzCore/Name/NameDictionary.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>

#include <utility>

namespace UnitTest
{

    class ModuleDataMock : public AZ::ModuleData
    {
    public:
        AZStd::shared_ptr<AZ::Entity> m_entity;
        AZStd::unique_ptr<AZ::DynamicModuleHandle> m_handle;
        AZ::Module* m_module = nullptr;

        explicit ModuleDataMock(AZStd::string name)
        {
            m_entity = AZStd::make_shared<AZ::Entity>();
            m_entity->SetName(name);
            LoadModule(name);
        }

        ~ModuleDataMock() override
        {
            UnloadModule();
            m_entity.reset();
        }

        [[nodiscard]] AZ::DynamicModuleHandle* GetDynamicModuleHandle() const override
        {
            return m_handle.get();
        }

        /// Get the handle to the module class
        [[nodiscard]] AZ::Module* GetModule() const override
        {
            return m_module;
        }

        /// Get the entity this module uses as a System Entity
        [[nodiscard]] AZ::Entity* GetEntity() const override
        {
            return m_entity.get();
        }

        /// Get the debug name of the module
        [[nodiscard]] const char* GetDebugName() const override
        {
            return m_entity->GetName().c_str();
        }

        void LoadModule(AZStd::string name)
        {
            m_handle = AZ::DynamicModuleHandle::Create(name.c_str());
            bool isLoaded = m_handle->Load(true);
            ASSERT_TRUE(isLoaded) << "Could not load required test module: "
                                  << m_handle->GetFilename(); // failed to load the DLL, please check the output paths

            auto createModule = m_handle->GetFunction<AZ::CreateModuleClassFunction>(AZ::CreateModuleClassFunctionName);
            // if this fails, we cannot continue as we will just nullptr exception
            ASSERT_NE(nullptr, createModule) << "Unable to find create module function in module: " << AZ::CreateModuleClassFunctionName;
            m_module = createModule();
            ASSERT_NE(nullptr, m_module);
        }

        void UnloadModule()
        {
            auto destroyModule = m_handle->GetFunction<AZ::DestroyModuleClassFunction>(AZ::DestroyModuleClassFunctionName);
            ASSERT_NE(nullptr, destroyModule) << "Could not find the destroy function in the module: "
                                              << AZ::DestroyModuleClassFunctionName;
            destroyModule(m_module);

            m_handle->Unload();
            m_handle.reset();
        }
    };
    class ModuleManagerRequestBusMock : public AZ::ModuleManagerRequestBus::Handler
    {
    public:
        void EnumerateModulesMock(AZ::ModuleManagerRequests::EnumerateModulesCallback callback)
        {
            auto data = ModuleDataMock("libAzCoreTestDLL.so");
            ASSERT_TRUE(callback(data));
        }

        ModuleManagerRequestBusMock()
        {
            AZ::ModuleManagerRequestBus::Handler::BusConnect();
            ON_CALL(*this, EnumerateModules(testing::_))
                .WillByDefault(testing::Invoke(this, &ModuleManagerRequestBusMock::EnumerateModulesMock));
        }

        ~ModuleManagerRequestBusMock() override
        {
            AZ::ModuleManagerRequestBus::Handler::BusDisconnect();
        }

        MOCK_METHOD1(EnumerateModules, void(AZ::ModuleManagerRequests::EnumerateModulesCallback perModuleCallback));
        MOCK_METHOD3(
            LoadDynamicModule,
            AZ::ModuleManagerRequests::LoadModuleOutcome(
                const char* modulePath, AZ::ModuleInitializationSteps lastStepToPerform, bool maintainReference));
        MOCK_METHOD3(
            LoadDynamicModules,
            AZ::ModuleManagerRequests::LoadModulesResult(
                const AZ::ModuleDescriptorList& modules, AZ::ModuleInitializationSteps lastStepToPerform, bool maintainReferences));
        MOCK_METHOD2(
            LoadStaticModules,
            AZ::ModuleManagerRequests::LoadModulesResult(
                AZ::CreateStaticModulesCallback staticModulesCb, AZ::ModuleInitializationSteps lastStepToPerform));
        MOCK_METHOD1(IsModuleLoaded, bool(const char* modulePath));
    };

    class WatchdogTest : public LeakDetectionFixture
    {
    public:
        void SetUp() override
        {
            LeakDetectionFixture::SetUp();
            AZ::NameDictionary::Create();

            m_sc = AZStd::make_unique<AZ::SerializeContext>();
            m_cd.reset(WatchdogTools::WatchdogToolsSystemComponent::CreateDescriptor());
            m_cd->Reflect(m_sc.get());

            m_settingsRegistry = AZStd::make_unique<AZ::SettingsRegistryImpl>();
            AZ::SettingsRegistry::Register(m_settingsRegistry.get());

            m_settingsRegistry->SetContext(m_sc.get());
            m_settingsRegistry->SetNotifyForMergeOperations(true);

            m_entity = aznew AZ::Entity();
            m_watchdogSystemComponent.reset(m_entity->CreateComponent<WatchdogTools::WatchdogToolsSystemComponent>());
        }

        void TearDown() override
        {
            m_entity->RemoveComponent(m_watchdogSystemComponent.get());
            m_watchdogSystemComponent.reset();
            delete m_entity;
            m_entity = nullptr;
            m_settingsRegistry.reset();
            m_cd.reset();
            m_sc.reset();

            AZ::NameDictionary::Destroy();
            LeakDetectionFixture::TearDown();
        }

        AZStd::unique_ptr<AZ::DynamicModuleHandle> m_handle;
        AZStd::unique_ptr<AZ::SerializeContext> m_sc;
        AZStd::unique_ptr<AZ::ComponentDescriptor> m_cd;

        AZStd::unique_ptr<AZ::SettingsRegistryImpl> m_settingsRegistry;
        AZStd::unique_ptr<WatchdogTools::WatchdogToolsSystemComponent> m_watchdogSystemComponent;
        AZ::Entity* m_entity;
        testing::NiceMock<ModuleManagerRequestBusMock> m_moduleManagerRequestBusMock;
    };

    TEST_F(WatchdogTest, EmptyRequiredModulesList)
    {
        m_entity->Init();
        m_entity->Activate();
        m_entity->Deactivate();
    }

    TEST_F(WatchdogTest, RequiredModuleNotLoaded)
    {
        constexpr AZStd::string_view RequiredModulesRegistry = R"(
        {
            "O3DE": {
                "Watchdog": {
                    "RequiredModules": [
                    "not_loaded_DLL"
                    ]
                }
            }
        }
        )";
        m_settingsRegistry->MergeSettings(RequiredModulesRegistry, AZ::SettingsRegistryInterface::Format::JsonMergePatch);

        m_entity->Init();
        EXPECT_EXIT(m_entity->Activate(), testing::KilledBySignal(SIGABRT), "terminate called without an active exception");
    }

    TEST_F(WatchdogTest, AllRequiredModulesLoaded_OnlyPrefix)
    {
        constexpr AZStd::string_view RequiredModulesRegistry = R"(
        {
            "O3DE": {
                "Watchdog": {
                    "RequiredModules": [
                    "libAzCoreTestDLL"
                    ]
                }
            }
        }
        )";
        m_settingsRegistry->MergeSettings(RequiredModulesRegistry, AZ::SettingsRegistryInterface::Format::JsonMergePatch);

        m_entity->Init();
        m_entity->Activate();
        m_entity->Deactivate();
    }

    TEST_F(WatchdogTest, AllRequiredModulesLoaded_FullName)
    {
        constexpr AZStd::string_view RequiredModulesRegistry = R"(
        {
            "O3DE": {
                "Watchdog": {
                    "RequiredModules": [
                    "libAzCoreTestDLL.so"
                    ]
                }
            }
        }
        )";
        m_settingsRegistry->MergeSettings(RequiredModulesRegistry, AZ::SettingsRegistryInterface::Format::JsonMergePatch);

        m_entity->Init();
        m_entity->Activate();
        m_entity->Deactivate();
    }

    TEST_F(WatchdogTest, AllRequiredModulesLoaded_NoPrefixNoSuffix)
    {
        constexpr AZStd::string_view RequiredModulesRegistry = R"(
        {
            "O3DE": {
                "Watchdog": {
                    "RequiredModules": [
                    "AzCoreTestDLL"
                    ]
                }
            }
        }
        )";
        m_settingsRegistry->MergeSettings(RequiredModulesRegistry, AZ::SettingsRegistryInterface::Format::JsonMergePatch);

        m_entity->Init();
        m_entity->Activate();
        m_entity->Deactivate();
    }

    TEST_F(WatchdogTest, AllRequiredModulesLoaded_NoPrefix)
    {
        constexpr AZStd::string_view RequiredModulesRegistry = R"(
        {
            "O3DE": {
                "Watchdog": {
                    "RequiredModules": [
                    "AzCoreTestDLL.so"
                    ]
                }
            }
        }
        )";
        m_settingsRegistry->MergeSettings(RequiredModulesRegistry, AZ::SettingsRegistryInterface::Format::JsonMergePatch);

        m_entity->Init();
        m_entity->Activate();
        m_entity->Deactivate();
    }
    TEST_F(WatchdogTest, AllRequiredModulesLoaded_NoSuffix)
    {
        constexpr AZStd::string_view RequiredModulesRegistry = R"(
        {
            "O3DE": {
                "Watchdog": {
                    "RequiredModules": [
                    "libAzCoreTestDLL"
                    ]
                }
            }
        }
        )";
        m_settingsRegistry->MergeSettings(RequiredModulesRegistry, AZ::SettingsRegistryInterface::Format::JsonMergePatch);

        m_entity->Init();
        m_entity->Activate();
        m_entity->Deactivate();
    }
} // namespace UnitTest

AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
