/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Color.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzTest/AzTest.h>
#include <Pointcloud/PointcloudAsset.h>
namespace UnitTest
{
    class PointcloudAssetTest : public LeakDetectionFixture
    {
        void SetUp() override
        {
            LeakDetectionFixture::SetUp();
            AZ::Data::AssetManager::Descriptor descriptor;
            AZ::Data::AssetManager::Create(descriptor);
            m_testHandler = AZStd::make_unique<Pointcloud::PointcloudAssetHandler>();
            AZ::Data::AssetManager::Instance().RegisterHandler(m_testHandler.get(), AZ::AzTypeInfo<Pointcloud::PointcloudAsset>::Uuid());
        }

        void TearDown() override
        {
            AZ::Data::AssetManager::Instance().DispatchEvents();
            AZ::Data::AssetManager::Instance().UnregisterHandler(m_testHandler.get());
            AZ::Data::AssetManager::Destroy();

            m_testHandler.reset();

            LeakDetectionFixture::TearDown();
        }

    protected:
        AZStd::unique_ptr<Pointcloud::PointcloudAssetHandler> m_testHandler;
    };

    //! Test the ability to write and read a product pointcloud asset.
    TEST_F(PointcloudAssetTest, WritingAndLoadingAsset)
    {
        AZ::Data::Asset<Pointcloud::PointcloudAsset> testAsset =
            AZ::Data::AssetManager::Instance().CreateAsset<Pointcloud::PointcloudAsset>(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        Pointcloud::PointcloudAsset& gold = *testAsset.Get();

        gold.m_data.resize(3);

        gold.m_data[0].m_position = { 1, 2, 3 };
        gold.m_data[0].m_color = AZ::Colors::Red.ToU32();
        gold.m_data[1].m_position = { 4, 5, 6 };
        gold.m_data[1].m_color = AZ::Colors::Green.ToU32();
        gold.m_data[2].m_position = { 7, 8, 9 };
        gold.m_data[2].m_color = AZ::Colors::Blue.ToU32();

        AZStd::vector<AZ::u8> buffer(1024);
        AZ::IO::MemoryStream stream(buffer.data(), buffer.size(), buffer.size());
        m_testHandler->SaveAssetData(testAsset, &stream);

        auto assetStream = AZStd::make_shared<AZ::Data::AssetDataStream>();
        assetStream->Open(buffer);
        AZ::Data::Asset<Pointcloud::PointcloudAsset> loadedAsset =
            AZ::Data::AssetManager::Instance().CreateAsset<Pointcloud::PointcloudAsset>(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        m_testHandler->LoadAssetData(loadedAsset, assetStream, nullptr);

        Pointcloud::PointcloudAsset& loaded = *loadedAsset.Get();
        ASSERT_EQ(loaded.m_data.size(), 3);
        ASSERT_EQ(loaded.m_data[0].m_position, gold.m_data[0].m_position);
        ASSERT_EQ(loaded.m_data[0].m_color, gold.m_data[0].m_color);
        ASSERT_EQ(loaded.m_data[1].m_position, gold.m_data[1].m_position);
        ASSERT_EQ(loaded.m_data[1].m_color, gold.m_data[1].m_color);
        ASSERT_EQ(loaded.m_data[2].m_position, gold.m_data[2].m_position);
        ASSERT_EQ(loaded.m_data[2].m_color, gold.m_data[2].m_color);
    }
    AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
} // namespace UnitTest
