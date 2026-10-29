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
uint32_t clockTime = 0;
uint32_t TestClock() { return clockTime; }
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
    if (currentScheduler) {
        currentScheduler->ScheduleTaskDeltaTime(taskA_id, 10);
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
        clockTime = 0;
        runCount = 0;
    }
};

// --- Unit Tests ---

// Tests whether the scheduler correctly assigns task ids until MaxTasks is reached
TEST_F(TaskSchedulerTest, TaskAllocation) {
    TaskScheduler<2, 1> tinyScheduler(TestClock);
    
    auto id1 = tinyScheduler.AddTask(TaskA_Callback, 0);
    auto id2 = tinyScheduler.AddTask(TaskB_Callback, 0);
    auto id3 = tinyScheduler.AddTask(TaskC_Callback, 0);
    
    EXPECT_TRUE(id1.has_value());
    EXPECT_TRUE(id2.has_value());
    EXPECT_FALSE(id3.has_value()) << "Third task should fail since array is full.";
}

// Tests whether tasks without dependencies simply execute once their timer expires
TEST_F(TaskSchedulerTest, BasicSchedulingAndExecution) {
    TaskScheduler<10, 2> scheduler(TestClock);
    auto id = scheduler.AddTask(TaskA_Callback, 0);
    ASSERT_TRUE(id.has_value());
    
    scheduler.ScheduleTaskTargetTime(*id, 100);
    
    clockTime = 50;
    scheduler.Run();
    EXPECT_FALSE(taskARan) << "Task ran before its scheduled time.";
    
    clockTime = 100;
    scheduler.Run();
    EXPECT_TRUE(taskARan) << "Task failed to run at its scheduled time.";
    
    // Verify state was cleared (it shouldn't run again unless rescheduled)
    taskARan = false;
    clockTime = 150;
    scheduler.Run();
    EXPECT_FALSE(taskARan) << "Task ran again without being rescheduled.";
}

// Tests whether the scheduler waits for dependencies to be met
// and immediately executes the task once they are
TEST_F(TaskSchedulerTest, Dependencies) {
    TaskScheduler<10, 2> scheduler(TestClock);
    auto id = scheduler.AddTask(TaskB_Callback, 2);
    ASSERT_TRUE(id.has_value());
    
    scheduler.ScheduleTaskTargetTime(*id, 200);
    
    // Time met, but 0 dependencies met
    clockTime = 200;
    scheduler.Run();
    EXPECT_FALSE(taskBRan);
    
    // 1 dependency met
    scheduler.SetDependency(*id, 0, true);
    clockTime = 200;
    scheduler.Run();
    EXPECT_FALSE(taskBRan);
    
    // All dependencies met
    scheduler.SetDependency(*id, 1, true);
    clockTime = 200;
    scheduler.Run();
    EXPECT_TRUE(taskBRan);
}

// Tests whether tasks are executed in the order their timers expired in
TEST_F(TaskSchedulerTest, ExecutionOrder) {
    TaskScheduler<10, 2> scheduler(TestClock);
    auto id1 = scheduler.AddTask(ExecutionOrder_CallbackA, 0);
    auto id2 = scheduler.AddTask(ExecutionOrder_CallbackB, 0);
    auto id3 = scheduler.AddTask(ExecutionOrder_CallbackC, 0);
    
    // Schedule completely out of chronological order
    scheduler.ScheduleTaskTargetTime(*id1, 30); // Runs 3rd
    scheduler.ScheduleTaskTargetTime(*id2, 10); // Runs 1st
    scheduler.ScheduleTaskTargetTime(*id3, 20); // Runs 2nd
    
    // Fast forward time past all scheduled times
    clockTime = 50;
    scheduler.Run();
    
    ASSERT_EQ(executionOrder.size(), 3);
    EXPECT_EQ(executionOrder[0], 2) << "Task B should have run first.";
    EXPECT_EQ(executionOrder[1], 3) << "Task C should have run second.";
    EXPECT_EQ(executionOrder[2], 1) << "Task A should have run third.";
}

// Tests whether self-rescheduling tasks run on time
TEST_F(TaskSchedulerTest, SelfRescheduling) {
    TaskScheduler<10, 2> scheduler(TestClock);
    currentScheduler = &scheduler; // Hook for the callback
    
    auto id = scheduler.AddTask(SelfRescheduling_Callback, 0);
    ASSERT_TRUE(id.has_value());
    taskA_id = *id;
    
    scheduler.ScheduleTaskTargetTime(taskA_id, 10);
    
    clockTime = 10;
    scheduler.Run();
    EXPECT_EQ(runCount, 1);
    
    clockTime = 15;
    scheduler.Run();
    EXPECT_EQ(runCount, 1) << "Ran prematurely after self-rescheduling.";
    
    clockTime = 20;
    scheduler.Run();
    EXPECT_EQ(runCount, 2) << "Failed to run after self-rescheduling.";
}

// Tests whether scheduler can handle tasks scheduled after timer rollover
// This is important since the scheduler can accept a time from any source,
// and we don't know how close we may be to a rollover
TEST_F(TaskSchedulerTest, TimerRollover) {
    TaskScheduler<10, 2> scheduler(TestClock);
    auto id = scheduler.AddTask(TaskC_Callback, 0);
    ASSERT_TRUE(id.has_value());
    
    // Schedule very close to 32-bit max
    uint32_t nearMax = 0xFFFFFFFA;
    scheduler.ScheduleTaskTargetTime(*id, nearMax);
    
    clockTime = nearMax - 5;
    scheduler.Run();
    EXPECT_FALSE(taskCRan);
    
    // Simulate timer wrapping around past 0
    clockTime = 0x00000005;
    scheduler.Run();
    EXPECT_TRUE(taskCRan) << "Rollover arithmetic failed.";
}

// Tests whether self-rescheduling tasks function properly when other tasks have unmet dependencies
TEST_F(TaskSchedulerTest, DependenciesWithSelfRescheduling) {
    TaskScheduler<10, 2> scheduler(TestClock);
    currentScheduler = &scheduler;

    auto idDep = scheduler.AddTask(TaskB_Callback, 1); // Unmet dependency
    auto idResched = scheduler.AddTask(SelfRescheduling_Callback, 0); // Self-reschedules
    ASSERT_TRUE(idDep.has_value());
    ASSERT_TRUE(idResched.has_value());
    taskA_id = *idResched;

    scheduler.ScheduleTaskTargetTime(*idDep, 10);
    scheduler.ScheduleTaskTargetTime(*idResched, 10);

    // At t=10: idDep has unmet dep, idResched runs and schedules for t=20
    clockTime = 10;
    scheduler.Run();
    EXPECT_EQ(runCount, 1);
    EXPECT_FALSE(taskBRan);

    // Satisfy dependency and run at t=20
    scheduler.SetDependency(*idDep, 0, true);
    clockTime = 20;
    scheduler.Run();
    EXPECT_EQ(runCount, 2);
    EXPECT_TRUE(taskBRan);
}

// The scheduler anchors periodic execution to the actual first dispatch, then
// preserves the deadline baseline even when later executions are delayed.
TEST_F(TaskSchedulerTest, DeltaSchedulingAnchorsFirstExecutionAndPreservesCadence) {
    TaskScheduler<10, 2> scheduler(TestClock);
    currentScheduler = &scheduler;
    taskA_id = *scheduler.AddTask(SelfRescheduling_Callback);

    clockTime = 100;
    scheduler.ScheduleTaskTargetTime(taskA_id, 100);
    clockTime = 125;
    scheduler.Run();
    EXPECT_EQ(runCount, 1);

    clockTime = 134;
    scheduler.Run();
    EXPECT_EQ(runCount, 1);
    clockTime = 138;
    scheduler.Run();
    EXPECT_EQ(runCount, 2);

    clockTime = 145;
    scheduler.Run();
    EXPECT_EQ(runCount, 3); // Previous deadline 135 + 10, rather than 138 + 10.
}

TEST_F(TaskSchedulerTest, DeltaSchedulingAccumulatesBeforeFirstExecution) {
    TaskScheduler<10, 2> scheduler(TestClock);
    auto id = *scheduler.AddTask(TaskA_Callback);
    clockTime = 100;
    scheduler.ScheduleTaskDeltaTime(id, 10);
    clockTime = 105;
    scheduler.ScheduleTaskDeltaTime(id, 10);

    clockTime = 119;
    scheduler.Run();
    EXPECT_FALSE(taskARan);
    clockTime = 120;
    scheduler.Run();
    EXPECT_TRUE(taskARan);
}

TEST_F(TaskSchedulerTest, DeltaSchedulingWrapsThroughZero) {
    TaskScheduler<10, 2> scheduler(TestClock);
    currentScheduler = &scheduler;
    taskA_id = *scheduler.AddTask(SelfRescheduling_Callback);
    clockTime = 0xFFFFFFF6U;
    scheduler.ScheduleTaskDeltaTime(taskA_id, 10);

    clockTime = 0xFFFFFFFFU;
    scheduler.Run();
    EXPECT_EQ(runCount, 0);
    clockTime = 0;
    scheduler.Run();
    EXPECT_EQ(runCount, 1);
    clockTime = 9;
    scheduler.Run();
    EXPECT_EQ(runCount, 1);
    clockTime = 10;
    scheduler.Run();
    EXPECT_EQ(runCount, 2);
}

TEST_F(TaskSchedulerTest, DeltaSchedulingCatchesUpOncePerPass) {
    TaskScheduler<10, 2> scheduler(TestClock);
    currentScheduler = &scheduler;
    taskA_id = *scheduler.AddTask(SelfRescheduling_Callback);
    clockTime = 0;
    scheduler.ScheduleTaskTargetTime(taskA_id, 0);
    scheduler.Run();
    EXPECT_EQ(runCount, 1);

    clockTime = 35;
    scheduler.Run();
    EXPECT_EQ(runCount, 2);
    scheduler.Run();
    EXPECT_EQ(runCount, 3);
    scheduler.Run();
    EXPECT_EQ(runCount, 4);
    scheduler.Run();
    EXPECT_EQ(runCount, 4);
    clockTime = 40;
    scheduler.Run();
    EXPECT_EQ(runCount, 5);
}
