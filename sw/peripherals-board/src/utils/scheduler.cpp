#include <scheduler.hpp>
#include <Arduino.h>

Scheduler::Scheduler(){}

void Scheduler::addTask(SchedulerTask task){

  _tasks.push_back(task);

}

void Scheduler::update(){

  for(SchedulerTask& task : _tasks){
    task.update();
  }
 
}

SchedulerTask::SchedulerTask(void(*taskFunction)(), unsigned long periodMS){

  _taskFunction = taskFunction;
  _periodMS = periodMS;
  _prevMillis = 0;

}

void SchedulerTask::update(){

  if(millis() - _prevMillis >= _periodMS){

    _prevMillis = millis();
    _taskFunction();

  }

}