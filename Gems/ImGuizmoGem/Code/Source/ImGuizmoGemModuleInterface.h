
#include <AzCore/Memory/Memory_fwd.h>
#include <AzCore/Module/Module.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/RTTI/TypeInfoSimple.h>

namespace ImGuizmoGem
{
    class ImGuizmoGemModuleInterface
        : public AZ::Module
    {
    public:
        AZ_TYPE_INFO_WITH_NAME_DECL(ImGuizmoGemModuleInterface)
        AZ_RTTI_NO_TYPE_INFO_DECL()
        AZ_CLASS_ALLOCATOR_DECL

        ImGuizmoGemModuleInterface();

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override;
    };
}// namespace ImGuizmoGem
