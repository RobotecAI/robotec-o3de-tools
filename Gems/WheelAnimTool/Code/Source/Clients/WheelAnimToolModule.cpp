/* Copyright 2025, Robotec.ai sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <WheelAnimTool/WheelAnimToolTypeIds.h>
#include <WheelAnimToolModuleInterface.h>

namespace WheelAnimTool
{
    class WheelAnimToolModule : public WheelAnimToolModuleInterface
    {
    public:
        AZ_RTTI(WheelAnimToolModule, WheelAnimToolModuleTypeId, WheelAnimToolModuleInterface);
        AZ_CLASS_ALLOCATOR(WheelAnimToolModule, AZ::SystemAllocator);
    };
} // namespace WheelAnimTool

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), WheelAnimTool::WheelAnimToolModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_WheelAnimTool, WheelAnimTool::WheelAnimToolModule)
#endif
