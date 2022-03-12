#ifndef HARDWARE_REMOTE_CONTROL_H
#define HARDWARE_REMOTE_CONTROL_H

// ROS libraries
#include <ros/ros.h>

// Control library
#include "vec6_hardwarecontroller.h"

// Misc libraries
#include "rc_config.h"
#include <thread>
#include <csignal>
#include "ncurses.h"

/// @brief Instance of the underwater vehicle (UWV)'s controller
Vec6HardwareController g_vec6;

/// @brief Indicates whether the user wishes to quit the ROS node or not
bool g_do_not_quit = true;

/**
 * @brief Displays descriptions of instructions recognisable by the
 * ROS node.
 */
void displayHelp(void);

/**
 * @brief Reads user's command and executes the corresponding function.
 */
void readInput(void);

/**
 * @brief Handles CTRL+C signal
 */ 
void signalHandler(int signum);

#endif
