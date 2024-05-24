#include "SplineToolsEditorComponent.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzToolsFramework/UI/UICore/WidgetHelpers.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <QFileDialog>
#include <QMessageBox>
#include <ROS2/Georeference/GeoreferenceBus.h>
#include <ROS2/Georeference/GeoreferenceStructures.h>
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
        ReloadCSVAsset();
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
        if (m_isLocalCoordinates && m_isLatLonAlt)
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
        try
        {
            AZStd::vector<AZ::Vector3> ret;

            csv::CSVReader reader(csvFilePath.c_str());
            reader.get_col_names();

            const int index_X = reader.index_of("x");
            const int index_Y = reader.index_of("y");
            const int index_Z = reader.index_of("z");

            const int index_Lat = reader.index_of("lat");
            const int index_Lon = reader.index_of("lon");
            const int index_Alt = reader.index_of("alt");

            const bool isCoordinateXY = !(index_X < 0 || index_Y < 0);
            const bool isCoordinateLatLon = !(index_Lat < 0 || index_Lon < 0 || index_Alt < 0);
            const bool isCoordinateCorrect = isCoordinateXY || isCoordinateLatLon;

            if (!isCoordinateCorrect)
            {
                AZ_Error("SplineToolsEditorComponent", false, "CSV file must have columns named x, y or lat, lon");
                QMessageBox::warning(
                    AzToolsFramework::GetActiveWindow(), "Error", "CSV file must have columns named x, y or lat, lon", QMessageBox::Ok);
                return {};
            }

            if (isCoordinateXY)
            {
                m_isLatLonAlt = false;
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
            }
            else if (isCoordinateLatLon)
            {
                m_isLatLonAlt = true;
                for (csv::CSVRow& row : reader)
                {
                    ROS2::WGS::WGS84Coordinate coordinate;
                    coordinate.m_latitude = row[index_Lat].get<double>();
                    coordinate.m_longitude = row[index_Lon].get<double>();
                    coordinate.m_altitude = row[index_Alt].get<float>();
                    AZ::Vector3 coordinateInLevel = AZ::Vector3(-1);
                    ROS2::GeoreferenceRequestsBus::BroadcastResult(
                        coordinateInLevel, &ROS2::GeoreferenceRequests::ConvertFromWSG84ToLevel, coordinate);

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
