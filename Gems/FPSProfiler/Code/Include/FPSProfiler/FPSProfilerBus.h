#pragma once

#include <Configurations/FPSProfilerConfig.h>
#include <FPSProfiler/FPSProfilerTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace FPSProfiler
{
    class FPSProfilerRequests
    {
    public:
        AZ_RTTI(FPSProfilerRequests, FPSProfilerRequestsTypeId);
        virtual ~FPSProfilerRequests() = default;

        /**
         * @brief Starts the profiling process to track system performance.
         *
         * Calls @ResetProfilingData and creates a csv file from provided path.
         * @attention You can change path with @ChangeSavePath or @SafeChangeSavePath.
         */
        virtual void StartProfiling() = 0;

        /**
         * @brief Stops the profiling process and saves data collection to provided path.
         * @attention You can change path with @ChangeSavePath or @SafeChangeSavePath.
         */
        virtual void StopProfiling() = 0;

        /**
         * @brief Resets all collected profiling data, clearing previous statistics.
         * @attention Reset is not calling @SaveLogToFile, it needs to be called manually.
         */
        virtual void ResetProfilingData() = 0;

        /**
         * @brief Checks whether profiling is currently active.
         * @return True if profiling is enabled, false otherwise.
         */
        [[nodiscard]] virtual bool IsProfiling() const = 0;

        /**
         * @brief Checks if any save option for profiling data is enabled.
         * @return True if at least one save option is active, false otherwise.
         */
        [[nodiscard]] virtual bool IsAnySaveOptionEnabled() const = 0;

        /**
         * @brief Changes the save path for profiling data.
         * @warning This function is NOT runtime safe. Use @ref SafeChangeSavePath instead.
         * @param newSavePath The new file path where profiling data should be saved.
         */
        virtual void ChangeSavePath(const AZStd::string& newSavePath) = 0;

        /**
         * @brief Safely changes the save path during runtime.
         * This method stops profiling, saves the current data, and then updates the path.
         * @param newSavePath The new file path where profiling data should be saved.
         */
        virtual void SafeChangeSavePath(const AZStd::string& newSavePath) = 0;

        /**
         * @brief Retrieves the minimum recorded FPS during the profiling session.
         * @return The lowest FPS value recorded at the moment of the call.
         */
        [[nodiscard]] virtual float GetMinFps() const = 0;

        /**
         * @brief Retrieves the maximum recorded FPS during the profiling session.
         * @return The highest FPS value recorded at the moment of the call.
         */
        [[nodiscard]] virtual float GetMaxFps() const = 0;

        /**
         * @brief Retrieves the average FPS over the profiling session.
         * @return The average FPS value at the moment of the call.
         */
        [[nodiscard]] virtual float GetAvgFps() const = 0;

        /**
         * @brief Retrieves the current real-time FPS value.
         * @return The FPS value at the moment of the call.
         */
        [[nodiscard]] virtual float GetCurrentFps() const = 0;

        /**
         * @brief Gets the current CPU memory usage.
         * @return A pair containing the currently used and reserved CPU memory (in bytes).
         */
        [[nodiscard]] virtual AZStd::pair<AZStd::size_t, AZStd::size_t> GetCpuMemoryUsed() const = 0;

        /**
         * @brief Gets the current GPU memory usage.
         * @return A pair containing the currently used and reserved GPU memory (in bytes).
         */
        [[nodiscard]] virtual AZStd::pair<AZStd::size_t, AZStd::size_t> GetGpuMemoryUsed() const = 0;

        /**
         * @brief Saves the current profiling log to a csv file at the predefined save path.
         */
        virtual void SaveLogToFile() = 0;

        /**
         * @brief Saves the profiling log to a specified file path.
         * @param newSavePath The csv file path where the log should be saved.
         * @param useSafeChangePath If true, the function will use @ref SafeChangeSavePath to ensure runtime safety.
         */
        virtual void SaveLogToFileWithNewPath(const AZStd::string& newSavePath, bool useSafeChangePath) = 0;

        /**
         * @brief Enables or disables FPS display on-screen.
         * @param enable If true, FPS will be displayed; otherwise, it will be hidden.
         */
        virtual void ShowFpsOnScreen(bool enable) = 0;
    };

    class FPSProfilerBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using FPSProfilerRequestBus = AZ::EBus<FPSProfilerRequests, FPSProfilerBusTraits>;
    using FPSProfilerInterface = AZ::Interface<FPSProfilerRequests>;

    // Notifications EBus (For Sending Events)
    class FPSProfilerNotifications
    {
    public:
        AZ_RTTI(FPSProfilerNotifications, FPSProfilerNotificationsTypeId);
        virtual ~FPSProfilerNotifications() = default;

        /**
         * @brief Called when a new file is created.
         * @param filePath The path of the newly created file.
         */
        virtual void OnFileCreated(const AZStd::string& filePath)
        {
        }

        /**
         * @brief Called when an existing file is updated.
         * @param filePath The path of the file that was modified.
         */
        virtual void OnFileUpdate(const AZStd::string& filePath)
        {
        }

        /**
         * @brief Called when a file is successfully saved.
         * @param filePath The path of the saved file.
         */
        virtual void OnFileSaved(const AZStd::string& filePath)
        {
        }

        /**
         * @brief Called when the profiling process starts.
         * @param config The configuration settings used for the profiling session.
         */
        virtual void OnProfileStart(const Configs::FileSaveSettings& config)
        {
        }

        /**
         * @brief Called when the profiling data is reset.
         * @param config The configuration settings used for the profiling session.
         */
        virtual void OnProfileReset(const Configs::FileSaveSettings& config)
        {
        }

        /**
         * @brief Called when the profiling process stops.
         * @param config The configuration settings used for the profiling session.
         */
        virtual void OnProfileStop(const Configs::FileSaveSettings& config)
        {
        }
    };

    class FPSProfilerNotificationBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple; // Allow multiple listeners
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single; // Only one global address
        //////////////////////////////////////////////////////////////////////////
    };

    using FPSProfilerNotificationBus = AZ::EBus<FPSProfilerNotifications, FPSProfilerNotificationBusTraits>;

    class FPSProfilerNotificationBusHandler
        : public FPSProfilerNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            FPSProfilerNotificationBusHandler,
            FPSProfilerNotificationBusHandlerTypeId,
            AZ::SystemAllocator,
            OnFileCreated,
            OnFileUpdate,
            OnFileSaved,
            OnProfileStart,
            OnProfileReset,
            OnProfileStop);

        void OnFileCreated(const AZStd::string& filePath) override
        {
            Call(FN_OnFileCreated, filePath);
        }

        void OnFileUpdate(const AZStd::string& filePath) override
        {
            Call(FN_OnFileUpdate, filePath);
        }

        void OnFileSaved(const AZStd::string& filePath) override
        {
            Call(FN_OnFileSaved, filePath);
        }

        void OnProfileStart(const Configs::FileSaveSettings& config) override
        {
            Call(FN_OnProfileStart, config);
        }

        void OnProfileReset(const Configs::FileSaveSettings& config) override
        {
            Call(FN_OnProfileReset, config);
        }

        void OnProfileStop(const Configs::FileSaveSettings& config) override
        {
            Call(FN_OnProfileStop, config);
        }
    };

} // namespace FPSProfiler
