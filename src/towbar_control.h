/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2024. All rights reserved.
 *
 * Towbar Control Module
 *
 * This module provides physics-based control for towbar-type tugs.
 * Towbar tugs differ from cradle tugs in that they use a rigid bar with
 * two articulation points (hinges):
 *   - Hinge A: At the tug's front hitch point
 *   - Hinge B: At the aircraft's nosewheel connection
 *
 * The towbar acts as a rigid link between these two points. When the tug
 * moves, the towbar articulates at both hinges, which affects the nosewheel
 * steering and ultimately the aircraft heading.
 *
 * Geometry (looking from above, aircraft facing up, tug facing down toward aircraft):
 *
 *          [AIRCRAFT]
 *              ^
 *              |
 *         [Nosewheel]---- Hinge B (towbar-nosewheel connection)
 *              |
 *        [===TOWBAR===]   (rigid bar, can articulate at both ends)
 *              |
 *         [Hitch]-------- Hinge A (tug-towbar hitch)
 *              |
 *           [TUG]
 *              v
 *
 * When pushing back:
 * - The tug drives FORWARD (toward aircraft, in tug's reference frame)
 * - This pushes the aircraft BACKWARD via the rigid towbar
 * - To turn the aircraft right (from aircraft's perspective), the tug must
 *   first steer left (from tug's perspective) to swing the nosewheel
 */

#ifndef _TOWBAR_CONTROL_H_
#define _TOWBAR_CONTROL_H_

#include <acfutils/geom.h>
#include <acfutils/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Towbar state structure
 * Tracks the current state of the towbar connection for physics calculations.
 */
typedef struct {
    /* Towbar geometry */
    double towbar_length;       /* Length of towbar in meters */
    double hitch_z;             /* Hitch position on tug (Z offset from origin) */

    /* Current articulation angles (degrees) */
    double hitch_angle;         /* Angle at hinge A (tug-towbar) */
    double nosewheel_angle;     /* Angle at hinge B (towbar-nosewheel) */

    /* Previous frame state for derivative calculations */
    double prev_tug_hdg;        /* Tug heading from previous frame */
    double prev_acf_hdg;        /* Aircraft heading from previous frame */
    double prev_hitch_angle;    /* Previous hitch angle */

    /* Towbar endpoint positions (world coordinates) */
    vect2_t hitch_pos;          /* Position of tug hitch in world coords */
    vect2_t nw_connection_pos;  /* Position of nosewheel connection */

    /* State flags */
    bool_t initialized;         /* Whether state has been initialized */
    bool_t connected;           /* Whether towbar is connected */
} towbar_state_t;

/*
 * Towbar control parameters
 * Tuning constants for the towbar physics simulation.
 */
typedef struct {
    double steer_response;      /* Steering response factor (0.0 - 1.0) */
    double hdg_damping;         /* Heading change damping factor */
    double max_hitch_angle;     /* Maximum articulation angle at hitch (degrees) */
    double max_nw_angle;        /* Maximum nosewheel steering angle (degrees) */
    double correction_gain;     /* Gain for steering correction */
} towbar_params_t;

/*
 * Input structure for towbar physics calculations
 */
typedef struct {
    /* Current aircraft state */
    vect2_t acf_pos;            /* Aircraft CG position (world coords) */
    double acf_hdg;             /* Aircraft heading (degrees) */
    double nw_z;                /* Nosewheel Z offset from CG (negative = forward) */
    double max_nw_steer;        /* Maximum nosewheel steering angle */

    /* Current tug state */
    vect2_t tug_pos;            /* Tug position (world coords) */
    double tug_hdg;             /* Tug heading (degrees) */
    double tug_speed;           /* Tug current speed (m/s) */
    double tug_cur_steer;       /* Tug current steering angle (degrees) */
    double tug_max_steer;       /* Tug maximum steering angle (degrees) */
    double tug_wheelbase;       /* Tug wheelbase (meters) */

    /* Desired steering */
    double req_steer;           /* Requested nosewheel steering angle (degrees) */

    /* Time step */
    double d_t;                 /* Delta time (seconds) */
} towbar_input_t;

/*
 * Output structure from towbar physics calculations
 */
typedef struct {
    /* Tug control outputs */
    double tug_steer_cmd;       /* Commanded tug steering angle (degrees) */
    vect2_t tug_new_pos;        /* New tug position (world coords) */
    double tug_new_hdg;         /* New tug heading (degrees) */

    /* Nosewheel output */
    double nw_steer;            /* Calculated nosewheel steering angle (degrees) */

    /* Aircraft heading correction */
    double acf_hdg_delta;       /* Heading correction to apply to aircraft (degrees) */

    /* Animation outputs */
    double towbar_heading;      /* Towbar heading for animation (degrees) */
    double towbar_pitch;        /* Towbar pitch for animation (degrees) */

    /* State outputs */
    double hitch_angle;         /* Current hitch articulation angle (degrees) */
    double connection_angle;    /* Current nosewheel connection angle (degrees) */
} towbar_output_t;

/*
 * Initialize towbar state
 * Must be called when a towbar tug is first connected to an aircraft.
 *
 * @param state     Pointer to towbar state structure to initialize
 * @param length    Towbar length in meters
 * @param hitch_z   Hitch Z position on tug (meters from origin)
 */
void towbar_state_init(towbar_state_t *state, double length, double hitch_z);

/*
 * Reset towbar state
 * Called when disconnecting towbar or resetting simulation.
 *
 * @param state     Pointer to towbar state structure to reset
 */
void towbar_state_reset(towbar_state_t *state);

/*
 * Get default towbar control parameters
 * Returns a towbar_params_t structure with reasonable default values.
 *
 * @return Default towbar parameters
 */
towbar_params_t towbar_get_default_params(void);

/*
 * Calculate towbar physics
 * Main physics calculation function. Computes tug steering, nosewheel angle,
 * and aircraft heading correction based on the current state and requested
 * steering.
 *
 * The physics model ensures:
 * 1. The towbar remains a rigid link (fixed length)
 * 2. Both hinge points maintain proper connection
 * 3. Forces are transmitted correctly through the towbar
 * 4. No unrealistic drift or disconnection occurs
 *
 * @param state     Current towbar state (updated on return)
 * @param params    Towbar control parameters
 * @param input     Current state inputs
 * @param output    Calculated outputs (filled on return)
 */
void towbar_calculate_physics(towbar_state_t *state,
                              const towbar_params_t *params,
                              const towbar_input_t *input,
                              towbar_output_t *output);

/*
 * Calculate tug position for towbar connection
 * Determines where the tug should be positioned to maintain proper towbar
 * connection to the aircraft nosewheel.
 *
 * @param state     Current towbar state
 * @param nw_pos    Aircraft nosewheel position (world coords)
 * @param nw_hdg    Nosewheel heading (degrees, based on aircraft heading)
 * @param tug_hdg   Current tug heading (degrees)
 * @param out_tug_pos   Output: calculated tug position
 * @param out_hitch_pos Output: calculated hitch position
 */
void towbar_calculate_tug_position(const towbar_state_t *state,
                                   vect2_t nw_pos,
                                   double nw_hdg,
                                   double tug_hdg,
                                   vect2_t *out_tug_pos,
                                   vect2_t *out_hitch_pos);

/*
 * Calculate required tug steering to achieve desired nosewheel angle
 * Given the current state and desired nosewheel steering angle, calculates
 * what steering input the tug should apply.
 *
 * @param state         Current towbar state
 * @param params        Towbar control parameters
 * @param cur_nw_steer  Current nosewheel steering angle (degrees)
 * @param req_nw_steer  Requested nosewheel steering angle (degrees)
 * @param tug_speed     Current tug speed (m/s)
 * @param d_t           Time step (seconds)
 * @return Required tug steering angle (degrees)
 */
double towbar_calculate_tug_steer(const towbar_state_t *state,
                                  const towbar_params_t *params,
                                  double cur_nw_steer,
                                  double req_nw_steer,
                                  double tug_speed,
                                  double d_t);

/*
 * Calculate nosewheel steering from towbar geometry
 * Determines the nosewheel steering angle based on the geometric relationship
 * between the tug heading, aircraft heading, and towbar orientation.
 *
 * @param acf_hdg   Aircraft heading (degrees)
 * @param tug_hdg   Tug heading (degrees)
 * @return Nosewheel steering angle (degrees, positive = right)
 */
double towbar_calculate_nw_steer(double acf_hdg, double tug_hdg);

/*
 * Calculate towbar articulation angles
 * Computes the angles at both hinge points of the towbar.
 *
 * @param state         Current towbar state (updated on return)
 * @param acf_hdg       Aircraft heading (degrees)
 * @param tug_hdg       Tug heading (degrees)
 * @param out_hitch_angle      Output: angle at hitch (degrees)
 * @param out_connection_angle Output: angle at nosewheel connection (degrees)
 */
void towbar_calculate_angles(towbar_state_t *state,
                             double acf_hdg,
                             double tug_hdg,
                             double *out_hitch_angle,
                             double *out_connection_angle);

/*
 * Validate towbar connection integrity
 * Checks if the towbar endpoints are within acceptable distance tolerances.
 * Returns false if the connection appears broken (endpoints too far apart
 * for the towbar length).
 *
 * @param state         Current towbar state
 * @param hitch_pos     Calculated hitch position
 * @param nw_pos        Nosewheel position
 * @param tolerance     Acceptable distance tolerance (meters)
 * @return true if connection is valid, false if broken
 */
bool_t towbar_validate_connection(const towbar_state_t *state,
                                  vect2_t hitch_pos,
                                  vect2_t nw_pos,
                                  double tolerance);

/*
 * Debug logging function for towbar physics
 * Logs current towbar state for debugging purposes.
 *
 * @param state     Current towbar state
 * @param input     Current inputs
 * @param output    Calculated outputs
 */
void towbar_debug_log(const towbar_state_t *state,
                      const towbar_input_t *input,
                      const towbar_output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* _TOWBAR_CONTROL_H_ */
