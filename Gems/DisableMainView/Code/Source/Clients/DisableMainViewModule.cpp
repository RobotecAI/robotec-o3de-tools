
#include <DisableMainView/DisableMainViewTypeIds.h>
#include <DisableMainViewModuleInterface.h>
#include "DisableMainViewSystemComponent.h"

namespace DisableMainView {
    class DisableMainViewModule
            : public DisableMainViewModuleInterface {
    public:
        AZ_RTTI(DisableMainViewModule, DisableMainViewModuleTypeId, DisableMainViewModuleInterface);

        AZ_CLASS_ALLOCATOR(DisableMainViewModule, AZ::SystemAllocator);
    };
}// namespace DisableMainView

AZ_DECLARE_MODULE_CLASS(Gem_DisableMainView, DisableMainView::DisableMainViewModule)
