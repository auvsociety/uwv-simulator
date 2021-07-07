#ifndef SIM_TASK_LIST_H
#define SIM_TASK_LIST_H

#include <ros/ros.h>

#include "vec6_simcontroller.h"
#include "vec6_config.h"
#include "tasks.h"

#ifdef USE_DARKNET
#include "darknet_ros_msgs/BoundingBoxes.h"
#endif

/**
 * @brief Abstract class defining and implementing methods and members common to tasks
 * that must be performed in simulation environment
 */ 
class SimTasks : public Tasks{
    protected:

    /// @brief Pointer to the controller
    Vec6SimController *vec6_;

    #ifdef USE_DARKNET
    /**
     * @brief Detect a particular object in the current camera frame.
     * 
     * @param _obj_detect A string representing the class of the obstacle to be detected.
     * @returns A boolean confirming if the given obstacle was detected or not.
     */
    bool detectObstacle(std::string _obj_detect){
        darknet_ros_msgs::BoundingBoxes objs_ = vec6_->vec6Comms.bbxs_;
        short size = objs_.bounding_boxes.size();

        ros::Duration(0.2).sleep();
        ros::spinOnce();

        objs_ = vec6_->vec6Comms.bbxs_;
        size = objs_.bounding_boxes.size();

        // Looking for obstacle in the current field of view (if probability > 0.5 --> detected)
        for(int i = 0; i < size; i++){
            if(objs_.bounding_boxes[i].Class.compare(_obj_detect) == 0){
                if(objs_.bounding_boxes[i].probability >= 0.5){
                    return true;
                }
            }
        }
        return false;
    } 
    #endif
};

/**
 * @brief Class defining the procedure to accomplish SAUVC task 1
 */ 
class SauvcTask1 : public SimTasks{
    public:
    /**
     * @brief Constructor
     * 
     * @param _controller Reference to the controller in use
     */ 
    SauvcTask1(Vec6SimController &_controller){
        vec6_ = &_controller;
    }

    /**
     * @brief Yaws and scans for obstacles
     */ 
    void scanObstacle();

	/**
	 * @brief Task 1 execution algorithm
	 */ 
	void execute() override;
};

/**
 * @brief Class defining a sample task.
 */ 
class Task1 : public SimTasks{
    public:
    /**
     * @brief Constructor
     * 
     * @param _controller Reference to the controller in use
     */ 
    Task1(Vec6SimController &_controller){
        vec6_ = &_controller;
    }

	/**
	 * @brief Task 1 execution algorithm
	 */ 
	void execute() override;
};

/**
 * @brief Class defining a sample task.
 */ 
class Task2 : public SimTasks{
    public:
    /**
     * @brief Constructor
     * 
     * @param _controller Reference to the controller in use
     */ 
    Task2(Vec6SimController &_controller){
        vec6_ = &_controller;
    }

	/**
	 * @brief Task 2 execution algorithm
	 */ 
	void execute() override;
};

/**
 * @brief Class defining a sample task.
 */ 
class Task3 : public SimTasks{
    public:
    /**
     * @brief Constructor
     * 
     * @param _controller Reference to the controller in use
     */ 
    Task3(Vec6SimController &_controller){
        vec6_ = &_controller;
    }

	/**
	 * @brief Task 3 execution algorithm
	 */ 
	void execute() override;
};


#endif