
#include <Billboard/BillboardTypeIds.h>
#include <BillboardModuleInterface.h>

namespace BillboardComponent
{
    class BillboardModule : public BillboardModuleInterface
    {
    public:
        AZ_RTTI(BillboardModule, BillboardModuleTypeId, BillboardModuleInterface);
        AZ_CLASS_ALLOCATOR(BillboardModule, AZ::SystemAllocator);
    };
} // namespace BillboardComponent

AZ_DECLARE_MODULE_CLASS(Gem_Billboard, BillboardComponent::BillboardModule)
