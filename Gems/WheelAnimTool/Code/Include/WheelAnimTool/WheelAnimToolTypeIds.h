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

#pragma once

namespace WheelAnimTool
{
    // System Component TypeIds
    inline constexpr const char* WheelAnimToolSystemComponentTypeId = "{43759813-052C-41F1-9B30-61BAE33C99C7}";
    inline constexpr const char* WheelAnimToolEditorSystemComponentTypeId = "{D7E9C79F-F9D1-4B02-A5AB-9F0CE0C7EE1C}";

    // Module derived classes TypeIds
    inline constexpr const char* WheelAnimToolModuleInterfaceTypeId = "{4E2DA4AC-7636-43A6-A935-F5B5560820BD}";
    inline constexpr const char* WheelAnimToolModuleTypeId = "{D1181477-468E-4948-8A5F-D68EA3B2A7A7}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* WheelAnimToolEditorModuleTypeId = WheelAnimToolModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* WheelAnimToolRequestsTypeId = "{B6EF8413-9B8A-411B-86D3-CF87AACDE064}";
} // namespace WheelAnimTool
