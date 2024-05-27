
#include <Pointcloud/PointcloudTypeIds.h>
#include <PointcloudModuleInterface.h>
#include "PointcloudSystemComponent.h"

#include <AzCore/RTTI/RTTI.h>

#include <Components/PointcloudComponent.h>

namespace Pointcloud
{
    class PointcloudModule
        : public PointcloudModuleInterface
    {
    public:
        AZ_RTTI(PointcloudModule, PointcloudModuleTypeId, PointcloudModuleInterface);
        AZ_CLASS_ALLOCATOR(PointcloudModule, AZ::SystemAllocator);

        PointcloudModule()
        {
            m_descriptors.insert(m_descriptors.end(),
                {
                    PointcloudSystemComponent::CreateDescriptor(),
                    PointcloudComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const
        {
            return AZ::ComponentTypeList{ azrtti_typeid<PointcloudSystemComponent>() };
        }
    };
}// namespace Pointcloud

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), Pointcloud::PointcloudModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_Pointcloud, Pointcloud::PointcloudModule)
#endif
