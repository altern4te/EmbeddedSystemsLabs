/**
 *  @file   maneuver_planner.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  Maneuver planner class header.
 *
 *  This file defines the maneuver planner class.
 */

/*
 *  Include guard.
 */
#ifndef PLANNER_MANEUVER_PLANNER_H_
#define PLANNER_MANEUVER_PLANNER_H_

/*
 *  External headers.
 */
#include <memory>
#include <vector>

/*
 *  Project headers.
 */
#include "planner/planner.h"

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
/*
 *  Forward declarations.
 */
struct ControllerReference;
struct Maneuver;

/**
 *  @brief  Maneuver planner class.
 *
 *  This class provides functions for creating and
 *  executing a maneuver-based plan. The class
 *  inherits the planner abstract class.
 */
class ManeuverPlanner : public Planner
{
public:

    /**
     *  @brief  Maneuver planner class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor defines a maneuver-based plan.
     */
    ManeuverPlanner();

    /**
     *  @brief  Start the defined maneuver-based plan.
     *
     *  This function starts a maneuver-based plan
     *  defined by the constructor. The function implements its
     *  pure abstract counterpart in the parent planner abstract
     *  class.
     */
    void IRAM_ATTR
    start() override;

    /**
     *  @return Maneuver counter.
     *  @brief  Execute the defined maneuver-based plan.
     *
     *  This function executes a maneuver-based plan defined by the
     *  constructor. The function implements its pure abstract counterpart
     *  in the parent planner abstract lass. This function is expected to
     *  be called periodically.
     */
    int
    plan() override;

private:

    /**
     *  @return Controller reference struct.
     *  @brief  Generate controller references from the current maneuver.
     *
     *  This function generates controller references from the current
     *  maneuver.
     */
    ControllerReference
    generateControllerReference() const;

    std::shared_ptr<Maneuver> maneuver_;    //!< Current maneuver shared pointer.
    std::shared_ptr<Maneuver> maneuver_start_;    //!< Start maneuver shared pointer.
    std::shared_ptr<Maneuver> avoid_left_;    //!< Avoidance maneuver for left obstacle.
    std::shared_ptr<Maneuver> avoid_right_;    //!< Avoidance maneuver for right obstacle.
    std::shared_ptr<Maneuver> avoid_middle_;    //!< Avoidance maneuver for middle obstacle.
    std::shared_ptr<Maneuver> drive_check_left_;    //!< Drive maneuver checking left sensor.
    std::shared_ptr<Maneuver> drive_check_right_;    //!< Drive maneuver checking right sensor.
    std::shared_ptr<Maneuver> drive_check_middle_;    //!< Drive maneuver checking middle sensor.
    std::shared_ptr<Maneuver> park_end_;    //!< Final park maneuver.
    int maneuver_counter_;  //!< Maneuver counter.
    unsigned long maneuver_timer_;  //!< Maneuver timer, in milliseconds.
    volatile bool plan_started_;    //!< Plan started flag.
    volatile bool maneuver_started_;    //!< Maneuver started flag.
    volatile bool plan_completed_;  //!< Plan completed flag.
};
}   // namespace firmware
}   // namespace biped

#endif  // PLANNER_MANEUVER_PLANNER_H_
