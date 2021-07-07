#include "tasks.h"

void TaskQueue::addTask(std::shared_ptr<Tasks> _task){
    pipeline_.push(_task);
}

void TaskQueue::executeFirstTask(){
    pipeline_.front()->execute();
    pipeline_.pop();
}