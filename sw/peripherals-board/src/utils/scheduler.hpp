/**
 * @brief Scheduler utility for managing periodic tasks with non-blocking execution
 * @author DIY Labs
 */

#pragma once

#include <vector>

/**
 * @brief Class representing a task in the scheduler. This task can be used directly, or added to a Scheduler.
 */
class SchedulerTask{
public:
  SchedulerTask(void(*)(), unsigned long periodMS);
  void update();

private:
  void (*_taskFunction)();
  unsigned long _periodMS;
  unsigned long _prevMillis;

};

/**
 * @brief Class for managing a collection of SchedulerTasks.
 */
class Scheduler{

public:
  Scheduler();

  void addTask(SchedulerTask task);
  void update();

private:  
  std::vector<SchedulerTask> _tasks;

};