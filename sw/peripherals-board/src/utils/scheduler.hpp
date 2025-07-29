#pragma once

#include <vector>

class SchedulerTask{
public:
  SchedulerTask(void(*)(), unsigned long periodMS);
  void update();

private:
  void (*_taskFunction)();
  unsigned long _periodMS;
  unsigned long _prevMillis;

};

class Scheduler{

public:
  Scheduler();

  void addTask(SchedulerTask task);
  void update();

private:  
  std::vector<SchedulerTask> _tasks;

};