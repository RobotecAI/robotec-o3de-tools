#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace SplineTools
{
    //! Editor component that allows to set Spline points from a CSV file
    //! The CSV file must have columns named x, y, z or lat, lon, alt
    //! Example of a CSV file:
    //! ```
    //! x,y,z
    //! 0,0,0
    //! 0,1,0
    //! -1,2,0
    //! 0,3,0
    //! -1,4,0
    //! 0,5,0
    //! ```

    class SplineToolsEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(SplineToolsEditorComponent, "{16bb03c4-d07f-4c06-b537-1a582e3c1802}");
        SplineToolsEditorComponent() = default;
        ~SplineToolsEditorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        AZ::IO::Path m_csvAssetPath; //!< Asset path of the CSV
        bool m_isLocalCoordinates = true;

        void ReloadCSVAsset();
        AZStd::vector<AZ::Vector3> GetSplinePointsFromCsv(const AZStd::string& csvFilePath);
        void SaveCsvAsset();
        void SavePointsToCsv(const AZStd::string& csvFilePath, const AZStd::vector<AZ::Vector3>& vertices);
    };
} // namespace SplineTools
