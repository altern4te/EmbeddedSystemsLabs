/**
 *  @file   waypoint_planner.cpp
 *  @author Simon Yu
 *  @date   01/20/2022
 *  @brief  Waypoint planner class source.
 *
 *  This file implements the waypoint planner class.
 */

/*
 *  Project headers.
 */
#include "common/global.h"
#include "common/type.h"
#include "controller/controller.h"
#include "planner/waypoint_planner.h"
#include "platform/serial.h"
#include "sensor/sensor.h"
#include "utility/math.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/*
 *  Firmware namespace.
 */
namespace firmware
{
WaypointPlanner::WaypointPlanner() : waypoint_counter_(1), waypoint_timer_(0), plan_started_(false),
        waypoint_started_(false), plan_completed_(true)
{
    /*
     *  Create a set of waypoints for the example plan.
     *  In the following configurations, the waypoints should
     *  be chained up in a linked list fashion.
     */
    std::shared_ptr<Waypoint> waypoint_1 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> waypoint_2 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> waypoint_3 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> waypoint_4 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> waypoint_5 = std::make_shared<Waypoint>();

    /*
     *  Set the start and current waypoints.
     */
    waypoint_start_ = waypoint_1;
    waypoint_ = waypoint_start_;

    /*
     *  Example plan waypoint 1 configuration:
     *      - Go to 0 meter X position (0 meters forward) without turning for 2 seconds.
     *      - Then, start waypoint 2.
     */
    waypoint_1->controller_reference.attitude_z = degreesToRadians(0);
    waypoint_1->controller_reference.position_x = 0;
    waypoint_1->duration = 2;
    waypoint_1->next = waypoint_2;

    /*
     *  Example plan waypoint 2 configuration:
     *      - Go to 1 meter X position (1 meter forward) without turning for 10 seconds.
     *      - Then, start waypoint 3.
     */
    waypoint_2->controller_reference.attitude_z = degreesToRadians(0);
    waypoint_2->controller_reference.position_x = 1;
    waypoint_2->duration = 10;
    waypoint_2->next = waypoint_3;

    /*
     *  Example plan waypoint 3 configuration:
     *      - Go to 2 meter X position (1 meter forward) while turning right 90 degrees for 10 seconds.
     *      - Then, start waypoint 4.
     */
    waypoint_3->controller_reference.attitude_z = degreesToRadians(90);
    waypoint_3->controller_reference.position_x = 2;
    waypoint_3->duration = 10;
    waypoint_3->next = waypoint_4;

    /*
     *  Example plan waypoint 4 configuration:
     *      - Go to 1 meter X position (1 meter backward) while turning left 90 degrees for 10 seconds.
     *      - Then, start waypoint 5.
     *
     *  Note that for the reverse turning directions, treat the reversing Biped as
     *  if it is driving forward. Then, the direction the Biped is turning in reverse
     *  is its reverse direction.
     */
    waypoint_4->controller_reference.attitude_z = degreesToRadians(-90);
    waypoint_4->controller_reference.position_x = 1;
    waypoint_4->duration = 10;
    waypoint_4->next = waypoint_5;

    /*
     *  Example plan waypoint 5 configuration:
     *      - Go to 0 meters X position (1 meter backward) without turning for 10 seconds.
     *      - The end.
     */
    waypoint_5->controller_reference.attitude_z = degreesToRadians(0);
    waypoint_5->controller_reference.position_x = 0;
    waypoint_5->duration = 10;
    waypoint_5->next = nullptr;

    /*
     *  Using the example plan above, create your own waypoint-based plan.
     *  Feel free to add or remove waypoints. Feel free to also remove or
     *  comment out the example plan.
     *
     *  Remember to initialize the class member start and current waypoint shared
     *  pointers to the first waypoint in your own maneuver-based plan.
     *
     *  TODO LAB 9 YOUR CODE HERE.
     */
    
    // Custom Lab 9 plan: Simulated obstacle avoidance path.
    // This waypoint plan demonstrates a path that could be used for obstacle avoidance.
    // The path moves forward, turns left to go around a potential left obstacle,
    // continues forward, straightens out, and returns to start position.
    // In a real dynamic system, sensor checks would adjust the path, but here
    // it's a predefined sequence simulating avoidance behavior.
    std::shared_ptr<Waypoint> wp1 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> wp2 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> wp3 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> wp4 = std::make_shared<Waypoint>();
    std::shared_ptr<Waypoint> wp5 = std::make_shared<Waypoint>();

    waypoint_start_ = wp1;
    waypoint_ = waypoint_start_;

    // Waypoint 1: Move to 0.4m forward, no turn, 5 seconds
    wp1->controller_reference.attitude_z = degreesToRadians(0);
    wp1->controller_reference.position_x = 0.4;
    wp1->duration = 5;
    wp1->next = wp2;

    // Waypoint 2: Turn left 45°, hold position, 3 seconds
    wp2->controller_reference.attitude_z = degreesToRadians(-45);  // Turn left
    wp2->controller_reference.position_x = 0.4;  // Hold position
    wp2->duration = 3;
    wp2->next = wp3;

    // Waypoint 3: Move to 0.8m forward, 45° turn, 8 seconds
    wp3->controller_reference.attitude_z = degreesToRadians(-45);  // Continue left turn
    wp3->controller_reference.position_x = 0.8;  // Move forward
    wp3->duration = 8;
    wp3->next = wp4;

    // Waypoint 4: Turn back to 0°, hold position, 3 seconds
    wp4->controller_reference.attitude_z = degreesToRadians(0);  // Straighten out
    wp4->controller_reference.position_x = 0.8;  // Hold position
    wp4->duration = 3;
    wp4->next = wp5;

    // Waypoint 5: Return to 0.4m, no turn, 8 seconds (end)
    wp5->controller_reference.attitude_z = degreesToRadians(0);
    wp5->controller_reference.position_x = 0.4;  // Return
    wp5->duration = 8;
    wp5->next = nullptr;   
}

void IRAM_ATTR
WaypointPlanner::start()
{
    /*
     *  If the plan is completed, reset the class member current waypoint
     *  shared pointer to the class member start waypoint shared pointer,
     *  reset the class member waypoint counter to 1, mark waypoint as not
     *  started, and mark the plan as not started and not completed.
     *
     *  TODO LAB 8 YOUR CODE HERE.
     */
    if (plan_completed_)
    {
        waypoint_ = waypoint_start_;
        waypoint_counter_ = 1;
        waypoint_started_ = false;
        plan_started_ = false;
        plan_completed_ = false;
    }
}

int
WaypointPlanner::plan()
{
    /*
     *  Validate controller global object shared pointer.
     */
    if (!controller_)
    {
        Serial(LogLevel::error) << "Controller missing.";
        return -1;
    }

    /*
     *  Validate sensor global object shared pointer.
     */
    if (!sensor_)
    {
        Serial(LogLevel::error) << "Sensor missing.";
        return -1;
    }

    /*
     *  Return -1 if the plan is completed or if the controller
     *  is not active (pause the plan during safety disengage).
     *
     *  TODO LAB 8 YOUR CODE HERE.
     */
    if (plan_completed_ || !controller_->getActiveStatus())
    {
        return -1;
    }

    /*
     *  Detect plan completion.
     */
    if (plan_started_ && !plan_completed_ && !waypoint_)
    {
        Serial(LogLevel::info) << "Completed waypoint-based plan.";

        /*
         *  Mark the plan as not started but completed and return
         *  -1 if the plan has started, has not completed, but the
         *  class member current waypoint shared pointer is a null
         *  pointer.
         *
         *  TODO LAB 8 YOUR CODE HERE.
         */
        plan_started_ = false;
        plan_completed_ = true;
        return -1;
    }

    if (!plan_started_)
    {
        /*
         *  The plan is empty if the plan has not started but the
         *  class member current waypoint shared pointer is already
         *  a null pointer. Return -1.
         */
        if (!waypoint_)
        {
            Serial(LogLevel::error) << "Empty waypoint-based plan.";
            return -1;
        }

        Serial(LogLevel::info) << "Started waypoint-based plan.";

        /*
         *  Otherwise, mark the plan as started if it has not started.
         *
         *  TODO LAB 8 YOUR CODE HERE.
         */
        plan_started_ = true;
    }

    if (!waypoint_started_)
    {
        Serial(LogLevel::info) << "Started waypoint " << waypoint_counter_ << ".";

        /*
         *  Start and execute the current waypoint if it has not started.
         *
         *  Set the controller reference in the current waypoint to the
         *  controller. Using the Arduino millis timing function, update the
         *  class member waypoint timer to the current time in milliseconds,
         *  and mark the current waypoint as started.
         *
         *  Refer to the controller header for the controller functions.
         *
         *  TODO LAB 8 YOUR CODE HERE.
         */
        ControllerReference ref = waypoint_->controller_reference;

        /*
         *  Adjust controller reference based on time-of-flight sensor data for obstacle avoidance.
         */
        TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
        if (tof_data.range_left < 0.1)
        {
            ref.attitude_z = degreesToRadians(45);  // Turn right to avoid left obstacle
        }
        else if (tof_data.range_right < 0.1)
        {
            ref.attitude_z = degreesToRadians(-45);  // Turn left to avoid right obstacle
        }
        else if (tof_data.range_middle < 0.1)
        {
            ref.position_x = sensor_->getEncoderData().position_x - 0.5;  // Reverse 0.5m to avoid middle obstacle
        }

        controller_->setControllerReference(ref);
        waypoint_timer_ = millis();
        waypoint_started_ = true;
    }
    else
    {
        /*
         *  If the elapsed duration, i.e., the difference between the current
         *  time in milliseconds given by the Arduino millis timing function and
         *  the class member waypoint timer, goes above the duration value in
         *  the current waypoint, transition to the next waypoint by setting the
         *  class member current waypoint shared pointer to the next waypoint
         *  shared pointer in the current waypoint, increment the class member
         *  waypoint counter, and mark the current waypoint as not started.
         *
         *  Remember to convert the duration value to the correct unit using
         *  the unit conversion functions in the math header.
         *
         *  Refer to the type header for the Waypoint struct entries.
         *
         *  TODO LAB 8 YOUR CODE HERE.
         */
        if (millis() - waypoint_timer_ >= static_cast<unsigned long>(waypoint_->duration * 1000))
        {
            waypoint_ = waypoint_->next;
            waypoint_counter_++;
            waypoint_started_ = false;
        }
    }

    /*
     *  Return the class member waypoint counter.
     *
     *  TODO LAB 8 YOUR CODE HERE.
     */
    return waypoint_counter_;
}
}   // namespace firmware
}   // namespace biped
