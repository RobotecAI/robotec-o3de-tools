
#include "ViewportStreamerSystemComponent.h"
#include <ViewportStreamer/ViewportStreamerTypeIds.h>
#include <ViewportStreamerModuleInterface.h>

namespace ViewportStreamer
{
    class ViewportStreamerModule : public ViewportStreamerModuleInterface
    {
    public:
        AZ_RTTI(ViewportStreamerModule, ViewportStreamerModuleTypeId, ViewportStreamerModuleInterface);
        AZ_CLASS_ALLOCATOR(ViewportStreamerModule, AZ::SystemAllocator);
    };
} // namespace ViewportStreamer

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ViewportStreamer::ViewportStreamerModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ViewportStreamer, ViewportStreamer::ViewportStreamerModule)
#endif
