#ifndef TASKS_H
#define TASKS_H

#include <queue>
#include <memory>

/**
 * @brief An abstract base class for implementing different tasks and mission planning for achieving autonomy 
 * of the vehicle. This class can be inherited easily and task-dependent functions can be added further.
 * Make sure to override the execute() function.
 */
class Tasks{
  public:
    /**
     * @brief Abstract method to be defined in the class describing the task.
     */ 
    virtual void execute() = 0;
};

class TaskQueue{
  public:

  /// @brief Queue of pointers to all the instances of derived classes of class Tasks
  std::queue<std::shared_ptr<Tasks>> pipeline_;
  
  /**
   * @brief Adds a task to the end task queue. 
   * Usage: task_queue.addTask(new Task1());
   * 
   * @param _task new instance of a derived class of class Tasks
   */ 
  void addTask(std::shared_ptr<Tasks> _task);

  /**
   * @brief Execute the first task present in the queue. The task is popped after
   * it's execution is completed.
   */ 
  void executeFirstTask();
  
};

#endif
