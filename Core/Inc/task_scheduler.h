/**
* @brief simple async task scheduler using a ring buffer of task IDs that the scheduler checks each loop.
* Each task may have dependencies, an execution time interval, and a callback function. The scheduler will
* wait until another callback/interrupt updates the dependencies as ready and the timer condition is met,
* then push the task ID to the execution queue.
*/

#pragma once

#include <cstdint>
#include <cstddef>
#include <optional>

#include "stm32f4xx_hal.h" 

typedef void (*TaskCallback)();

template <size_t MaxTasks, size_t QueueSize>
class TaskScheduler {
public:
    struct Task {
        TaskCallback callback = nullptr;
        uint32_t intervalMs = 0;
        uint32_t lastScheduledTime = 0;
        bool hasDependencies = false;
        
        // volatile flags modified by asynchronous isrs
        volatile bool timeReady = false;
        volatile bool depsReady = false;
        bool active = false;
    };

    // static pointer instance to interface with C
    static TaskScheduler* Instance;

    TaskScheduler() {
        Instance = this;
    }

    /**
     * @brief automatically allocates a task to the next available open slot
     * @param cb: task callback to execute when both the timer and hardware are ready
     * @param intervalMs: how often the timer should attempt to rechedule the task
     * @param hasDeps: whether the task has other dependencies that the scheduler should wait for (defaults to false);
     * the dependent task's callback is responsible for updating whether the dependencies have been met
     * @retval std::optional containing the auto-generated id, or nullopt if array is full
     */
    std::optional<size_t> AddTask(TaskCallback cb, uint32_t intervalMs, bool hasDeps = false) {
        for (size_t i = 0; i < MaxTasks; i++) {
            if (!_tasks[i].active) {
                _tasks[i].callback = cb;
                _tasks[i].intervalMs = intervalMs;
                _tasks[i].lastScheduledTime = 0; 
                _tasks[i].hasDependencies = hasDeps;
                
                // reset internal state machine
                _tasks[i].timeReady = false;
                _tasks[i].depsReady = false;
                _tasks[i].active = true;
                
                return i;
            }
        }
        return std::nullopt; // array is full
    }

    /**
     * @brief advances the internal timer one tick; should be called in HAL_SYSTICK_Callback()
     * @param currentTickMs: the current hardware tick from HAL_GetTick()
     */
    void AdvanceSysTickTimer(uint32_t currentTickMs) {
        for (size_t i = 0; i < MaxTasks; i++) {
            // inactive tasks aren't checked until they are marked active again
            if (_tasks[i].active && !_tasks[i].timeReady) {
                // unsigned subtraction handles timer rollover
                if ((uint32_t)(currentTickMs - _tasks[i].lastScheduledTime) >= _tasks[i].intervalMs) {
                    _tasks[i].timeReady = true;
                    
                    // if dependencies and timer are ready, add to task execution queue
                    if (!_tasks[i].hasDependencies || _tasks[i].depsReady) {
                        PushToQueue(i);
                    }
                }
            }
        }
    }

    /**
     * @brief marks dependencies as ready for the specified task; should be called from hardware interrupts
     * when depencency tasks finish
     * @param id: the id of the task to mark as ready
     */
    void SetDependenciesFulfilled(size_t id) {
        if (id < MaxTasks && _tasks[id].active) {
            _tasks[id].depsReady = true;
            
            // if the timer already popped, queue it immediately
            if (_tasks[id].timeReady) {
                PushToQueue(id);
            }
        }
    }

    /**
     * @brief main execution step that reads from ring buffer of ready tasks; should be called in a
     * superloop in the calling task
     */
    void Run() {
        size_t taskId;
        
        if (PopFromQueue(taskId)) {
            Task& t = _tasks[taskId];
            
            if (t.callback) {
                t.callback();
            }

            // reset state machine for the next cycle
            t.timeReady = false;
            t.depsReady = false;
            
            // prevent timer drift by marking actual scheduled time instead of when it was supposed to run
            t.lastScheduledTime += t.intervalMs; 
        }
    }

private:
    Task _tasks[MaxTasks];
    
    volatile size_t _head = 0;
    volatile size_t _tail = 0;
    volatile size_t _queue[QueueSize];

    /**
     * @brief adds a ready task to the task execution queue
     * @param id: the id of the task to queue
     */
    void PushToQueue(size_t id) {
        // uses a ring buffer with modulo arithmetic to prevent blocking or allocating new arrays
        size_t nextHead = (_head + 1) % QueueSize;
        // if the queue is full, the task will be skipped here and queued in the next 
        if (nextHead != _tail) { 
            _queue[_head] = id;
            // data memory barrier ensures data is saved before head changes
            __DMB();             
            _head = nextHead;
        }
    }

    /**
     * @brief removes a completed task from the execution queue
     * @param id: the id of the task to remove from the queue
     * @retval bool indicating whether there was a task in the queue to execute; false means
     * the queue is empty.
     */
    bool PopFromQueue(size_t& id) {
        if (_head == _tail) return false; 
        
        id = _queue[_tail];
        // data memory barrier ensures data is read before tail changes
        __DMB();                 
        _tail = (_tail + 1) % QueueSize;
        
        return true;
    }
};

template <size_t MaxTasks, size_t QueueSize>
TaskScheduler<MaxTasks, QueueSize>* TaskScheduler<MaxTasks, QueueSize>::Instance = nullptr;