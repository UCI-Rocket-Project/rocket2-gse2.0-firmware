#include <gtest/gtest.h>
#include <mutex>
#include <vector>

#include "../RocketDrivers/task_scheduler/task_scheduler.h"

// Define the simulated critical section mutex for the PC build
std::mutex test_mutex;

// --- Global state ---

bool taskARan = false;
bool taskBRan = false;
bool taskCRan = false;
std::vector<int> executionOrder;

// Global pointers/state for the self-rescheduling test
TaskScheduler<10, 2>* currentScheduler = nullptr;
size_t taskA_id = 0;
uint32_t selfRescheduleTarget = 10;
int runCount = 0;

// Task callbacks
void TaskA_Callback() { taskARan = true; }
void TaskB_Callback() { taskBRan = true; }
void TaskC_Callback() { taskCRan = true; }

void ExecutionOrder_CallbackA() { executionOrder.push_back(1); }
void ExecutionOrder_CallbackB() { executionOrder.push_back(2); }
void ExecutionOrder_CallbackC() { executionOrder.push_back(3); }

void SelfRescheduling_Callback() {
    runCount++;
    selfRescheduleTarget += 10;
    if (currentScheduler) {
        currentScheduler->ScheduleTask(taskA_id, selfRescheduleTarget);
    }
}

// Test fixture - runs before every test to clear global state
class TaskSchedulerTest : public ::testing::Test {
protected:
    void SetUp() override {
        taskARan = false;
        taskBRan = false;
        taskCRan = false;
        executionOrder.clear();
        
        currentScheduler = nullptr;
        taskA_id = 0;
        selfRescheduleTarget = 10;
        runCount = 0;
    }
};

// --- Unit Tests ---

// Tests whether the scheduler correctly assigns task ids until MaxTasks is reached
TEST_F(TaskSchedulerTest, TaskAllocation) {
    TaskScheduler<2, 1> tinyScheduler;
    
    auto id1 = tinyScheduler.AddTask(TaskA_Callback, 0);
    auto id2 = tinyScheduler.AddTask(TaskB_Callback, 0);
    auto id3 = tinyScheduler.AddTask(TaskC_Callback, 0);
    
    EXPECT_TRUE(id1.has_value());
    EXPECT_TRUE(id2.has_value());
    EXPECT_FALSE(id3.has_value()) << "Third task should fail since array is full.";
}

// Tests whether tasks without dependencies simply execute once their timer expires
TEST_F(TaskSchedulerTest, BasicSchedulingAndExecution) {
    TaskScheduler<10, 2> scheduler;
    auto id = scheduler.AddTask(TaskA_Callback, 0);
    ASSERT_TRUE(id.has_value());
    
    scheduler.ScheduleTask(*id, 100);
    
    scheduler.Run(50);
    EXPECT_FALSE(taskARan) << "Task ran before its scheduled time.";
    
    scheduler.Run(100);
    EXPECT_TRUE(taskARan) << "Task failed to run at its scheduled time.";
    
    // Verify state was cleared (it shouldn't run again unless rescheduled)
    taskARan = false;
    scheduler.Run(150);
    EXPECT_FALSE(taskARan) << "Task ran again without being rescheduled.";
}

// Tests whether the scheduler waits for dependencies to be met
// and immediately executes the task once they are
TEST_F(TaskSchedulerTest, Dependencies) {
    TaskScheduler<10, 2> scheduler;
    auto id = scheduler.AddTask(TaskB_Callback, 2);
    ASSERT_TRUE(id.has_value());
    
    scheduler.ScheduleTask(*id, 200);
    
    // Time met, but 0 dependencies met
    scheduler.Run(200);
    EXPECT_FALSE(taskBRan);
    
    // 1 dependency met
    scheduler.SetDependency(*id, 0, true);
    scheduler.Run(200);
    EXPECT_FALSE(taskBRan);
    
    // All dependencies met
    scheduler.SetDependency(*id, 1, true);
    scheduler.Run(200);
    EXPECT_TRUE(taskBRan);
}

// Tests whether tasks are executed in the order their timers expired in
TEST_F(TaskSchedulerTest, ExecutionOrder) {
    TaskScheduler<10, 2> scheduler;
    auto id1 = scheduler.AddTask(ExecutionOrder_CallbackA, 0);
    auto id2 = scheduler.AddTask(ExecutionOrder_CallbackB, 0);
    auto id3 = scheduler.AddTask(ExecutionOrder_CallbackC, 0);
    
    // Schedule completely out of chronological order
    scheduler.ScheduleTask(*id1, 30); // Runs 3rd
    scheduler.ScheduleTask(*id2, 10); // Runs 1st
    scheduler.ScheduleTask(*id3, 20); // Runs 2nd
    
    // Fast forward time past all scheduled times
    scheduler.Run(50);
    
    ASSERT_EQ(executionOrder.size(), 3);
    EXPECT_EQ(executionOrder[0], 2) << "Task B should have run first.";
    EXPECT_EQ(executionOrder[1], 3) << "Task C should have run second.";
    EXPECT_EQ(executionOrder[2], 1) << "Task A should have run third.";
}

// Tests whether self-rescheduling tasks run on time
TEST_F(TaskSchedulerTest, SelfRescheduling) {
    TaskScheduler<10, 2> scheduler;
    currentScheduler = &scheduler; // Hook for the callback
    
    auto id = scheduler.AddTask(SelfRescheduling_Callback, 0);
    ASSERT_TRUE(id.has_value());
    taskA_id = *id;
    
    scheduler.ScheduleTask(taskA_id, 10);
    
    scheduler.Run(10);
    EXPECT_EQ(runCount, 1);
    
    scheduler.Run(15);
    EXPECT_EQ(runCount, 1) << "Ran prematurely after self-rescheduling.";
    
    scheduler.Run(20);
    EXPECT_EQ(runCount, 2) << "Failed to run after self-rescheduling.";
}

// Tests whether scheduler can handle tasks scheduled after timer rollover
// This is important since the scheduler can accept a time from any source,
// and we don't know how close we may be to a rollover
TEST_F(TaskSchedulerTest, TimerRollover) {
    TaskScheduler<10, 2> scheduler;
    auto id = scheduler.AddTask(TaskC_Callback, 0);
    ASSERT_TRUE(id.has_value());
    
    // Schedule very close to 32-bit max
    uint32_t nearMax = 0xFFFFFFFA;
    scheduler.ScheduleTask(*id, nearMax);
    
    scheduler.Run(nearMax - 5);
    EXPECT_FALSE(taskCRan);
    
    // Simulate timer wrapping around past 0
    scheduler.Run(0x00000005); 
    EXPECT_TRUE(taskCRan) << "Rollover arithmetic failed.";
}