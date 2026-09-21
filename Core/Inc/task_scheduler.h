#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <optional>

// Mock the interrupt enable/disable for testing purposes
#ifdef __arm__
    #include "stm32f1xx_hal.h"
    #define SCHEDULER_CRITICAL_ENTER() uint32_t primask = __get_PRIMASK(); __disable_irq()
    #define SCHEDULER_CRITICAL_EXIT()  __set_PRIMASK(primask)
#else
    #include <mutex>
    extern std::mutex test_mutex;
    #define SCHEDULER_CRITICAL_ENTER() test_mutex.lock()
    #define SCHEDULER_CRITICAL_EXIT()  test_mutex.unlock()
#endif

// A task defines its callback as a generic function pointer
typedef void (*TaskCallback)();

template <size_t MaxTasks, size_t MaxDepsPerTask>
class TaskScheduler {
public:
    struct Task {
        TaskCallback callback = nullptr;
        // Tasks schedule themselves for certain timestamps
        uint32_t scheduledTime = 0;
        
        std::array<bool, MaxDepsPerTask> dependencies = {false};
        // The number of required dependencies to check for this task
        // Must be <= `MaxDepsPerTask`; dependencies beyond this number are not checked
        size_t requiredDependencies = 0; 
        
        // Whether the task is scheduled to execute again
        bool active = false;
        // Index of the next task in the linked list
        size_t next = MaxTasks; 
    };

    TaskScheduler() {}

    /**
     * @brief Allocates a task to the next available open slot.
     * @param cb The function to execute.
     * @param requiredDeps How many dependencies this specific task needs (up to `MaxDepsPerTask`).
     * @retval The task id, if created successfully, or std::nullopt otherwise.
     */
    std::optional<size_t> AddTask(TaskCallback cb, size_t requiredDeps = 0) {
        if (requiredDeps > MaxDepsPerTask) return std::nullopt;

        for (size_t i = 0; i < MaxTasks; i++) {
            if (!_tasks[i].callback) {
                _tasks[i].callback = cb;
                _tasks[i].requiredDependencies = requiredDeps;
                _tasks[i].active = false;
                _tasks[i].next = MaxTasks;
                
                for (size_t d = 0; d < MaxDepsPerTask; ++d) {
                    _tasks[i].dependencies[d] = false;
                }
                return i;
            }
        }
        return std::nullopt; 
    }

    /**
     * @brief Safely sorts a task by execution time into the execution list.
     * @param id The id of the task to schedule
     * @param targetTime When the task should attempt to be executed; this is the time
     * that's used to insert the task into the list in its sorted position
     */
    void ScheduleTask(size_t id, uint32_t targetTime) {
        if (id >= MaxTasks || !_tasks[id].callback) return;

        // Disable interrupts temporarily to prevent superloop modifying task at the same time
        SCHEDULER_CRITICAL_ENTER();

        RemoveFromList(id);

        _tasks[id].scheduledTime = targetTime;
        _tasks[id].active = true;
        _tasks[id].next = MaxTasks;

        // If list is empty OR targetTime is chronologically BEFORE the head's time,
        // move the new task to the front
        if (_head == MaxTasks || TimeIsBefore(targetTime, _tasks[_head].scheduledTime)) {
            _tasks[id].next = _head;
            _head = id;
        } else {
            size_t curr = _head;
            
            // Iterate until we find a task scheduled LATER than our target time
            while (_tasks[curr].next != MaxTasks && 
                   !TimeIsBefore(targetTime, _tasks[_tasks[curr].next].scheduledTime)) {
                curr = _tasks[curr].next;
            }
            _tasks[id].next = _tasks[curr].next;
            _tasks[curr].next = id;
        }

        // Re-enable interrupts
        SCHEDULER_CRITICAL_EXIT();
    }

    /**
     * @brief Sets the state of a single task dependency.
     * @param id The task id, assigned on creation.
     * @param depIndex The index of the specific dependency in the task's array.
     * @param state Whether the dependency has been met or not.
     */
    void SetDependency(size_t id, size_t depIndex, bool state) {
        if (id < MaxTasks && depIndex < _tasks[id].requiredDependencies) {
            _tasks[id].dependencies[depIndex] = state;
        }
    }

    /**
     * @brief Runs all ready tasks in a single cycle, evaluating timers and dependencies, 
     * @param currentTime The time to compare the scheduled time of the tasks against.
     */
    void Run(uint32_t currentTime) {
        size_t curr = _head;
        size_t prev = MaxTasks;

        // Iterates over the sorted linked list of tasks until finding the first task
        // whose time hasn't passed yet
        // Executes all tasks for which the time and dependency requirements have been met
        while (curr != MaxTasks) {
            Task& t = _tasks[curr];

            // Bail out instantly if the current time has not reached the scheduled time
            // This prevents us from checking any future tasks after this one
            if (!TimeIsReady(currentTime, t.scheduledTime)) {
                break;
            }

            // Only check the active dependencies for this specific task
            bool depsMet = true;
            for (size_t i = 0; i < t.requiredDependencies; ++i) {
                if (!t.dependencies[i]) {
                    depsMet = false;
                    break;
                }
            }

            if (depsMet) {
                // Disable interrupts temporarily to prevent superloop modifying task at the same time
                SCHEDULER_CRITICAL_ENTER();
                size_t nextNode = t.next;
                
                // Remove task from the linked list
                if (prev == MaxTasks) {
                    _head = nextNode;
                } else {
                    _tasks[prev].next = nextNode;
                }

                // Re-enable interrupts
                SCHEDULER_CRITICAL_EXIT();

                // Clear state to prevent race conditions during self-rescheduling
                t.active = false;
                t.next = MaxTasks;
                for (size_t i = 0; i < t.requiredDependencies; ++i) {
                    t.dependencies[i] = false;
                }

                // Execute the task
                t.callback();

                // If the task was executed, we don't update prev since that task is not active now
                curr = nextNode;
            } else {
                // Time is met but dependencies aren't; keep iterating
                prev = curr;
                curr = t.next;
            }
        }
    }

private:
    std::array<Task, MaxTasks> _tasks;
    volatile size_t _head = MaxTasks;

    /**
     * @brief Compares two timestamps, handling hardware timer rollovers.
     * Evaluates true if t1 is chronologically before t2.
     */
    static inline bool TimeIsBefore(uint32_t t1, uint32_t t2) {
        return (int32_t)(t1 - t2) < 0;
    }

    /**
     * @brief Evaluates true if the current time has reached or passed the scheduled time.
     */
    static inline bool TimeIsReady(uint32_t current, uint32_t scheduled) {
        return (int32_t)(current - scheduled) >= 0;
    }

    /**
     * @brief Helper function to remove a task from the linked list of scheduled tasks.
     */
    void RemoveFromList(size_t id) {
        if (_head == MaxTasks) return;
        if (_head == id) {
            _head = _tasks[id].next;
            return;
        }
        size_t curr = _head;
        while (_tasks[curr].next != MaxTasks) {
            if (_tasks[curr].next == id) {
                _tasks[curr].next = _tasks[id].next;
                return;
            }
            curr = _tasks[curr].next;
        }
    }
};