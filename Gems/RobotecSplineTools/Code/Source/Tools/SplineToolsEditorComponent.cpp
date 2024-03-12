#include "SplineToolsEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <csv/csv.hpp>

namespace RobotecSplineTools
{

    void SplineToolsEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void SplineToolsEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<SplineToolsEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("CsvAssetId", &SplineToolsEditorComponent::m_csvAssetId)
                ->Field("IsLocalCoordinates", &SplineToolsEditorComponent::m_isLocalCoordinates);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<SplineToolsEditorComponent>("SplineToolsEditorComponent", "SplineToolsEditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "SplineToolsEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::CheckBox,
                        &SplineToolsEditorComponent::m_isLocalCoordinates,
                        "Local coordinates",
                        "Local coordinates")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SplineToolsEditorComponent::m_csvAssetId, "CSV Asset", "CSV asset")
                    ->Attribute(AZ::Edit::Attributes::SourceAssetFilterPattern, "*.csv")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Reload spline", "Reload spline")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Load")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &SplineToolsEditorComponent::ReloadCSVAsset)
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "");
            }
        }
    }

    void SplineToolsEditorComponent::Activate()
    {
    }

    void SplineToolsEditorComponent::Deactivate()
    {
    }

    void SplineToolsEditorComponent::BuildGameEntity([[maybe_unused]] AZ::Entity* gameEntity)
    {
        ReloadCSVAsset();
    }

    void SplineToolsEditorComponent::ReloadCSVAsset()
    {
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        AZ::Data::AssetInfo sourceAssetInfo;
        bool ok{ false };
        AZStd::string watchFolder;
        AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;

        AssetSysReqBus::BroadcastResult(
            ok, &AssetSysReqBus::Events::GetSourceInfoBySourceUUID, m_csvAssetId.m_guid, sourceAssetInfo, watchFolder);
        const AZ::IO::Path sourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(sourceAssetInfo.m_relativePath);

        AZ_Printf("SplineToolsEditorComponent", "Reload csv asset from %s", sourcePath.c_str());

        auto points = GetSplinePointsFromCsv(sourcePath.c_str());

        if (points.empty())
        {
            AZ_Error("TreeSpawnerEditorComponent", false, "No points found in CSV file");
            return;
        }

        AZ::Transform worldTM(AZ::Transform::Identity());
        if (!m_isLocalCoordinates)
        {
            AZ::TransformBus::EventResult(worldTM, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        }

        LmbrCentral::SplineComponentRequestBus::Event(GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::ClearVertices);
        AZ_Printf("SplineToolsEditorComponent", "CSV file has %d points", points.size());

        for (auto p : points)
        {
            p = worldTM.TransformPoint(p);
            LmbrCentral::SplineComponentRequestBus::Event(GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::AddVertex, p);
        }
    }

    AZStd::vector<AZ::Vector3> SplineToolsEditorComponent::GetSplinePointsFromCsv(const AZStd::string& csvFilePath)
    {
        try
        {
            AZStd::vector<AZ::Vector3> ret;

            csv::CSVReader reader(csvFilePath.c_str());
            reader.get_col_names();

            const int index_X = reader.index_of("x");
            const int index_Y = reader.index_of("y");
            const int index_Z = reader.index_of("z");

            const bool isCoordinateCorrect = !(index_X < 0 || index_Y < 0);
            if (!isCoordinateCorrect)
            {
                AZ_Error("TreeSpawnerEditorComponent", false, "CSV file must have columns named x, y");
                return {};
            }

            for (csv::CSVRow& row : reader)
            {
                AZ::Vector3 point = AZ::Vector3(row[index_X].get<float>(), row[index_Y].get<float>(), 0);

                // handle Z column
                if (index_Z > 0)
                {
                    point.SetZ(row[index_Z].get<float>());
                }

                ret.emplace_back(AZStd::move(point));
            }

            return ret;
        } catch (std::runtime_error& exception)
        {
            AZ_Error("TreeSpawnerEditorComponent", false, "Error parsing CSV file: %s", exception.what());
        }
        return {};
    }

} // namespace RobotecSplineTools
