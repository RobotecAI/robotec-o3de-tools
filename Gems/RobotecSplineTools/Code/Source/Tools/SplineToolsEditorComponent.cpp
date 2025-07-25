#include "SplineToolsEditorComponent.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzToolsFramework/UI/UICore/WidgetHelpers.h>
#include <Georeferencing/GeoreferenceBus.h>
#include <Georeferencing/GeoreferenceStructures.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <QFileDialog>
#include <QMessageBox>
#include <csv/csv.hpp>

namespace SplineTools
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
            serializeContext->Class<SplineToolsEditorComponent, AzToolsFramework::Components::EditorComponentBase>()->Version(2)->Field(
                "IsLocalCoordinates", &SplineToolsEditorComponent::m_isLocalCoordinates);

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
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Load spline", "Load spline")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Load")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &SplineToolsEditorComponent::ReloadCSVAsset)
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Save spline", "Save spline")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Save")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &SplineToolsEditorComponent::SaveCsvAsset)
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
    }

    void SplineToolsEditorComponent::SaveCsvAsset()
    {
        AZ::SplinePtr splinePtr;
        LmbrCentral::SplineComponentRequestBus::EventResult(
            splinePtr, GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::GetSpline);
        auto vertices = splinePtr->GetVertices();

        QString fileName = QFileDialog::getSaveFileName(AzToolsFramework::GetActiveWindow(), "Open CSV File", "", "CSV Files (*.csv)");

        if (fileName.isEmpty())
        {
            QMessageBox::warning(AzToolsFramework::GetActiveWindow(), "Error", "Please specify file", QMessageBox::Ok);
            return;
        }

        const AZ::IO::Path sourcePath = AZ::IO::Path(fileName.toUtf8().constData());

        if (!m_isLocalCoordinates)
        {
            auto worldTm{ AZ::Transform::Identity() };

            AZ::TransformBus::EventResult(worldTm, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
            AZStd::transform(
                vertices.begin(),
                vertices.end(),
                vertices.begin(),
                [worldTm](auto point)
                {
                    return worldTm.TransformPoint(point);
                });
        }
        AZ_Printf("SplineToolsEditorComponent", "Save CSV asset to %s", sourcePath.c_str());
        SavePointsToCsv(sourcePath.c_str(), vertices);
    }

    void SplineToolsEditorComponent::ReloadCSVAsset()
    {
        QString fileName = QFileDialog::getOpenFileName(AzToolsFramework::GetActiveWindow(), "Open CSV File", "", "CSV Files (*.csv)");

        if (fileName.isEmpty())
        {
            QMessageBox::warning(AzToolsFramework::GetActiveWindow(), "Error", "Please specify file", QMessageBox::Ok);
            return;
        }
        AZStd::string sourcePath = AZStd::string(fileName.toUtf8().constData());
        AZ_Printf("SplineToolsEditorComponent", "Reload csv asset from %s", sourcePath.c_str());

        auto points = GetSplinePointsFromCsv(sourcePath.c_str());

        if (points.empty())
        {
            AZ_Error("SplineToolsEditorComponent", false, "No points found in CSV file");
            QMessageBox::warning(AzToolsFramework::GetActiveWindow(), "Error", "No points found in CSV file", QMessageBox::Ok);
            return;
        }

        AZ::Transform worldInvTm(AZ::Transform::Identity());
        if (m_isLocalCoordinates && m_isCoordinateLatLon)
        {
            AZ_Error("SplineToolsEditorComponent", false, "Cannot use lat, lon, alt coordinates in local coordinates");
            QMessageBox::warning(
                AzToolsFramework::GetActiveWindow(), "Error", "Cannot use lat, lon, alt coordinates in local coordinates", QMessageBox::Ok);
            return;
        }

        if (!m_isLocalCoordinates)
        {
            AZ::TransformBus::EventResult(worldInvTm, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
            worldInvTm.Invert();
        }

        LmbrCentral::SplineComponentRequestBus::Event(GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::ClearVertices);
        AZ_Printf("SplineToolsEditorComponent", "CSV file has %d points", points.size());

        AZStd::transform(
            points.begin(),
            points.end(),
            points.begin(),
            [&worldInvTm](AZ::Vector3& p)
            {
                return worldInvTm.TransformPoint(p);
            });

        LmbrCentral::SplineComponentRequestBus::Event(GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::SetVertices, points);
    }

    void SplineToolsEditorComponent::SavePointsToCsv(const AZStd::string& csvFilePath, const AZStd::vector<AZ::Vector3>& vertices)
    {
        try
        {
            auto fileStream = std::ofstream(csvFilePath.c_str());

            if (!fileStream.is_open())
            {
                AZ_Error("SplineToolsEditorComponent", false, "Could not open file for writing - (%s).", csvFilePath.c_str());
                return;
            }
            auto writer = csv::make_csv_writer(fileStream);
            writer << std::make_tuple("x", "y", "z");
            for (auto& vertex : vertices)
            {
                writer << std::make_tuple(vertex.GetX(), vertex.GetY(), vertex.GetZ());
            }
            writer.flush();
            AZ_Info("SplineToolsEditorComponent", "Saved %zu points to CSV file", vertices.size());
        } catch (std::runtime_error& exception)
        {
            AZ_Error("SplineToolsEditorComponent", false, "Error saving CSV file: %s", exception.what());
        }
    }

    AZStd::vector<AZ::Vector3> SplineToolsEditorComponent::GetSplinePointsFromCsv(const AZStd::string& csvFilePath)
    {
        auto getOptionalColumn = [](int column) -> AZStd::optional<int>
        {
            if (column < 0)
            {
                return AZStd::nullopt;
            }
            return column;
        };

        try
        {
            AZStd::vector<AZ::Vector3> ret;

            csv::CSVReader reader(csvFilePath.c_str());
            reader.get_col_names();

            const auto indexX = getOptionalColumn(reader.index_of("x"));
            const auto indexY = getOptionalColumn(reader.index_of("y"));
            const auto indexZ = getOptionalColumn(reader.index_of("z"));

            const auto indexLat = getOptionalColumn(reader.index_of("lat"));
            const auto indexLon = getOptionalColumn(reader.index_of("lon"));
            const auto indexAlt = getOptionalColumn(reader.index_of("alt"));

            m_isCoordinateXY = indexX && indexY;
            m_isCoordinateLatLon = indexLat && indexLon && indexAlt;
            const bool isCoordinateCorrect = (m_isCoordinateLatLon || m_isCoordinateXY);

            if (!isCoordinateCorrect)
            {
                AZ_Error("SplineToolsEditorComponent", false, "CSV file must have columns named x, y or lat, lon, alt");
                AZStd::string columns;
                for (const auto& columnName : reader.get_col_names())
                {
                    columns += AZStd::string(columnName.c_str()) + ", ";
                }
                AZ_Printf("SplineToolsEditorComponent", "CSV file columns: %s", columns.c_str());
                QMessageBox::warning(
                    AzToolsFramework::GetActiveWindow(),
                    "Error",
                    "CSV file must have columns named x, y or lat, lon, alt",
                    QMessageBox::Ok);
                return {};
            }

            if (m_isCoordinateXY)
            {
                for (csv::CSVRow& row : reader)
                {
                    AZ::Vector3 point = AZ::Vector3(row[*indexX].get<float>(), row[*indexY].get<float>(), 0);

                    if (indexZ > 0)
                    {
                        point.SetZ(row[*indexZ].get<float>());
                    }

                    ret.emplace_back(AZStd::move(point));
                }
            }
            else if (m_isCoordinateLatLon)
            {
                for (csv::CSVRow& row : reader)
                {
                    using namespace Georeferencing;
                    WGS::WGS84Coordinate coordinate;
                    coordinate.m_latitude = row[*indexLat].get<double>();
                    coordinate.m_longitude = row[*indexLon].get<double>();
                    coordinate.m_altitude = row[*indexAlt].get<float>();
                    auto coordinateInLevel = AZ::Vector3(-1);
                    GeoreferenceRequestsBus::BroadcastResult(coordinateInLevel, &GeoreferenceRequests::ConvertFromWGS84ToLevel, coordinate);

                    ret.emplace_back(AZStd::move(coordinateInLevel));
                }
            }

            return ret;
        } catch (std::runtime_error& exception)
        {
            AZStd::string error = AZStd::string::format("Error parsing CSV file: %s", exception.what());
            AZ_Error("SplineToolsEditorComponent", false, error.c_str());
            QMessageBox::warning(AzToolsFramework::GetActiveWindow(), "Error", error.c_str(), QMessageBox::Ok);
        }
        return {};
    }

} // namespace SplineTools
