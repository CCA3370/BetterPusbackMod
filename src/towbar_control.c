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
 * Towbar Control Module - Implementation
 *
 * This module implements physics-based control for towbar-type tugs.
 *
 * PHYSICS MODEL OVERVIEW:
 *
 * The towbar is modeled as a rigid bar with two articulation points:
 *
 *   1. Hinge A (at tug): The towbar connects to the tug at a hitch point.
 *      This hinge allows the towbar to rotate horizontally relative to
 *      the tug's longitudinal axis.
 *
 *   2. Hinge B (at aircraft): The towbar connects to the aircraft's
 *      nosewheel. This connection point follows the nosewheel's steering
 *      angle, which is constrained by the aircraft's steering limits.
 *
 * KEY PHYSICAL CONSTRAINTS:
 *
 *   - The towbar length is fixed (rigid bar)
 *   - Both hinge points must remain connected
 *   - The nosewheel steering is limited by aircraft design
 *   - The hitch angle is limited by tug design
 *
 * CONTROL STRATEGY:
 *
 * When pushing back (tug driving forward, aircraft moving backward):
 *
 *   To turn aircraft RIGHT (from aircraft's perspective):
 *   1. Tug steers LEFT (from tug's perspective, steering wheel turns left)
 *   2. This swings the tug's rear to the RIGHT
 *   3. The hitch point moves RIGHT
 *   4. The towbar rotates clockwise (looking from above)
 *   5. The nosewheel is pushed to steer RIGHT
 *   6. Aircraft turns RIGHT while moving backward
 *
 * The nosewheel steering angle is geometrically determined by:
 *   nw_steer = rel_hdg(acf_hdg + 180, tug_hdg)
 *
 * This represents the angle the towbar makes with the aircraft's
 * backward direction, which directly corresponds to nosewheel steering.
 */

#include <math.h>
#include <string.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/math.h>

#include "towbar_control.h"
#include "xplane.h"

/*
 * Physical constants and tuning parameters
 */

/* Maximum rate of change for steering (degrees per second) */
#define TOWBAR_MAX_STEER_RATE       30.0

/* Steering correction gain - how aggressively to correct steering errors */
#define TOWBAR_STEER_GAIN           0.5

/* Heading damping - prevents oscillation in heading corrections */
#define TOWBAR_HDG_DAMPING          0.15

/* Heading amplification - scales heading correction for physics engine */
#define TOWBAR_HDG_AMPLIFY          3.0

/* Minimum speed for steering updates (m/s) */
#define TOWBAR_MIN_SPEED            0.01

/* Maximum articulation angle at hitch (degrees) */
#define TOWBAR_MAX_HITCH_ANGLE      75.0

/* Connection tolerance for validation (meters) */
#define TOWBAR_CONNECTION_TOLERANCE 0.5

/*
 * Helper function: Normalize heading to 0-360 range
 */
static double
normalize_heading(double hdg)
{
    while (hdg >= 360.0)
        hdg -= 360.0;
    while (hdg < 0.0)
        hdg += 360.0;
    return hdg;
}

/*
 * Helper function: Calculate relative heading (shortest angular distance)
 * Returns the angle from hdg1 to hdg2, in range [-180, 180]
 */
static double
relative_heading(double hdg1, double hdg2)
{
    double diff = hdg2 - hdg1;
    while (diff > 180.0)
        diff -= 360.0;
    while (diff < -180.0)
        diff += 360.0;
    return diff;
}

/*
 * Helper function: Convert heading to direction vector
 */
static vect2_t
heading_to_dir(double hdg)
{
    double rad = DEG2RAD(hdg);
    return VECT2(sin(rad), cos(rad));
}

/*
 * Helper function: Clamp value to range
 */
static double
clamp_value(double val, double min_val, double max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

void
towbar_state_init(towbar_state_t *state, double length, double hitch_z)
{
    ASSERT(state != NULL);
    ASSERT(length > 0);

    memset(state, 0, sizeof(*state));
    state->towbar_length = length;
    state->hitch_z = hitch_z;
    state->initialized = B_TRUE;
    state->connected = B_FALSE;
    state->hitch_angle = 0;
    state->nosewheel_angle = 0;
    state->prev_tug_hdg = 0;
    state->prev_acf_hdg = 0;
    state->prev_hitch_angle = 0;
    state->hitch_pos = ZERO_VECT2;
    state->nw_connection_pos = ZERO_VECT2;
}

void
towbar_state_reset(towbar_state_t *state)
{
    ASSERT(state != NULL);
    
    state->connected = B_FALSE;
    state->hitch_angle = 0;
    state->nosewheel_angle = 0;
    state->prev_tug_hdg = 0;
    state->prev_acf_hdg = 0;
    state->prev_hitch_angle = 0;
    state->hitch_pos = ZERO_VECT2;
    state->nw_connection_pos = ZERO_VECT2;
}

towbar_params_t
towbar_get_default_params(void)
{
    towbar_params_t params;
    params.steer_response = TOWBAR_STEER_GAIN;
    params.hdg_damping = TOWBAR_HDG_DAMPING;
    params.max_hitch_angle = TOWBAR_MAX_HITCH_ANGLE;
    params.max_nw_angle = 75.0;  /* Will be overridden by aircraft limits */
    params.correction_gain = 1.0;
    return params;
}

double
towbar_calculate_nw_steer(double acf_hdg, double tug_hdg)
{
    /*
     * The nosewheel steering angle for a towbar tug is determined by
     * the angle between the tug's heading and the aircraft's backward
     * direction.
     *
     * When the tug is directly behind the aircraft (aligned for straight
     * pushback), tug_hdg = acf_hdg + 180, so the relative angle is 0.
     *
     * When the tug rotates right (tug_hdg increases), the nosewheel
     * steers right (positive angle).
     */
    double acf_backward = normalize_heading(acf_hdg + 180.0);
    return relative_heading(acf_backward, tug_hdg);
}

void
towbar_calculate_angles(towbar_state_t *state,
                        double acf_hdg,
                        double tug_hdg,
                        double *out_hitch_angle,
                        double *out_connection_angle)
{
    ASSERT(state != NULL);
    
    /*
     * The towbar forms angles at both hinge points.
     *
     * At the hitch (Hinge A):
     * The angle between the tug's forward direction and the towbar.
     * Since the towbar points from the hitch toward the nosewheel,
     * and the tug faces the aircraft, this angle represents how much
     * the towbar has rotated from the tug's longitudinal axis.
     *
     * At the nosewheel connection (Hinge B):
     * The angle between the aircraft's backward direction and the towbar.
     * This is equivalent to the nosewheel steering angle.
     */
    
    /* Calculate the nosewheel steering angle (same as towbar-aircraft angle) */
    double nw_steer = towbar_calculate_nw_steer(acf_hdg, tug_hdg);
    
    /*
     * The hitch angle and nosewheel angle are related through the
     * geometry of the rigid towbar. For a simplified model where both
     * articulate equally (symmetric), each hinge takes half the total
     * angular difference.
     *
     * Total angular difference = tug_hdg - (acf_hdg + 180)
     * Hitch angle = Connection angle = Total / 2
     *
     * However, in practice, the nosewheel has steering limits that may
     * be different from the hitch limits. We use the nosewheel angle
     * directly for the connection, and calculate the hitch angle based
     * on the remaining articulation needed.
     */
    double hitch_angle = nw_steer / 2.0;
    double connection_angle = nw_steer / 2.0;
    
    /* Update state */
    state->hitch_angle = hitch_angle;
    state->nosewheel_angle = nw_steer;
    
    if (out_hitch_angle != NULL)
        *out_hitch_angle = hitch_angle;
    if (out_connection_angle != NULL)
        *out_connection_angle = connection_angle;
}

void
towbar_calculate_tug_position(const towbar_state_t *state,
                              vect2_t nw_pos,
                              double nw_hdg,
                              double tug_hdg,
                              vect2_t *out_tug_pos,
                              vect2_t *out_hitch_pos)
{
    ASSERT(state != NULL);
    ASSERT(state->initialized);
    
    /*
     * Calculate tug position to maintain proper towbar connection.
     *
     * The geometry is:
     *   [Nosewheel] ---(towbar_length)--- [Hitch] ---(hitch_z)--- [Tug origin]
     *
     * The towbar points from the nosewheel toward the hitch. The direction
     * of the towbar is determined by the articulation at both hinges.
     *
     * For symmetric articulation, the towbar direction is halfway between
     * the aircraft's backward direction and the tug's forward direction.
     */
    
    /* Calculate the towbar direction */
    /* The towbar articulates at both ends, so its direction is the average */
    double acf_backward = normalize_heading(nw_hdg + 180.0);
    double rel_angle = relative_heading(acf_backward, tug_hdg);
    double towbar_hdg = normalize_heading(acf_backward + rel_angle / 2.0);
    
    vect2_t towbar_dir = heading_to_dir(towbar_hdg);
    
    /* Hitch position is towbar_length from nosewheel in towbar direction */
    /* Note: We use the aircraft forward direction (nw_hdg) as reference */
    /* Since the tug is in front of the aircraft (in acf forward direction), */
    /* we add the towbar length in the aircraft forward direction, adjusted */
    /* by the articulation angle */
    vect2_t hitch_pos = vect2_add(nw_pos, 
                                   vect2_scmul(towbar_dir, state->towbar_length));
    
    /* Tug direction (facing toward aircraft, heading = acf_hdg + 180 + steer) */
    vect2_t tug_dir = heading_to_dir(tug_hdg);
    
    /* Tug origin is hitch_z distance from hitch, opposite to tug's forward dir */
    /* hitch_z is positive when hitch is in front of tug origin */
    vect2_t tug_pos = vect2_add(hitch_pos, 
                                 vect2_scmul(tug_dir, -state->hitch_z));
    
    if (out_tug_pos != NULL)
        *out_tug_pos = tug_pos;
    if (out_hitch_pos != NULL)
        *out_hitch_pos = hitch_pos;
}

double
towbar_calculate_tug_steer(const towbar_state_t *state,
                           const towbar_params_t *params,
                           double cur_nw_steer,
                           double req_nw_steer,
                           double tug_speed,
                           double d_t)
{
    ASSERT(state != NULL);
    ASSERT(params != NULL);
    ASSERT(d_t > 0);
    
    /*
     * Calculate the tug steering required to achieve the desired nosewheel
     * steering angle.
     *
     * The relationship between tug steering and nosewheel steering is:
     *
     * For towbar tugs during pushback (tug driving forward):
     * - Tug steers LEFT -> nosewheel steers RIGHT
     * - Tug steers RIGHT -> nosewheel steers LEFT
     *
     * This is because:
     * - When the tug's front wheels turn left, the tug's rear swings right
     * - The hitch (at the front of the tug) moves left
     * - This pulls the towbar left
     * - The nosewheel is pushed to point right
     *
     * The correction is proportional to the steering error.
     */
    
    double steer_error = req_nw_steer - cur_nw_steer;
    
    /* Apply correction gain */
    double correction = steer_error * params->steer_response;
    
    /*
     * Direction multiplier:
     * - When pushing back (tug speed > 0, tug driving toward aircraft):
     *   Positive steer_error (want more right) -> negative tug steer (steer left)
     * - When towing forward (tug speed < 0):
     *   Direction is reversed
     */
    double dir_mult = (tug_speed >= 0) ? -1.0 : 1.0;
    
    double tug_steer = dir_mult * correction;
    
    /* Limit rate of steering change */
    double max_change = TOWBAR_MAX_STEER_RATE * d_t;
    tug_steer = clamp_value(tug_steer, -max_change, max_change);
    
    return tug_steer;
}

bool_t
towbar_validate_connection(const towbar_state_t *state,
                           vect2_t hitch_pos,
                           vect2_t nw_pos,
                           double tolerance)
{
    ASSERT(state != NULL);
    
    double actual_dist = vect2_dist(hitch_pos, nw_pos);
    double expected_dist = state->towbar_length;
    
    return (fabs(actual_dist - expected_dist) <= tolerance);
}

void
towbar_calculate_physics(towbar_state_t *state,
                         const towbar_params_t *params,
                         const towbar_input_t *input,
                         towbar_output_t *output)
{
    ASSERT(state != NULL);
    ASSERT(params != NULL);
    ASSERT(input != NULL);
    ASSERT(output != NULL);
    
    /* Initialize output to safe defaults */
    memset(output, 0, sizeof(*output));
    
    /*
     * STEP 1: Calculate nosewheel position in world coordinates
     */
    vect2_t acf_dir = heading_to_dir(input->acf_hdg);
    vect2_t nw_pos = vect2_add(input->acf_pos, 
                                vect2_scmul(acf_dir, -input->nw_z));
    
    /*
     * STEP 2: Calculate current nosewheel steering from geometry
     * The nosewheel steering is the angle between the tug's heading
     * and the aircraft's backward direction.
     */
    double cur_nw_steer = towbar_calculate_nw_steer(input->acf_hdg, 
                                                     input->tug_hdg);
    
    /* Limit to aircraft steering limits */
    cur_nw_steer = clamp_value(cur_nw_steer, 
                               -input->max_nw_steer, 
                               input->max_nw_steer);
    
    /*
     * STEP 3: Calculate articulation angles
     */
    double hitch_angle, connection_angle;
    towbar_calculate_angles(state, input->acf_hdg, input->tug_hdg,
                           &hitch_angle, &connection_angle);
    
    /*
     * STEP 4: Calculate required tug steering to achieve desired nosewheel angle
     */
    double req_steer = clamp_value(input->req_steer,
                                   -input->max_nw_steer,
                                   input->max_nw_steer);
    
    double tug_steer_cmd = 0;
    if (fabs(input->tug_speed) > TOWBAR_MIN_SPEED) {
        tug_steer_cmd = towbar_calculate_tug_steer(state, params,
                                                    cur_nw_steer, req_steer,
                                                    input->tug_speed,
                                                    input->d_t);
        
        /* Add current steering as base */
        tug_steer_cmd += input->tug_cur_steer;
        
        /* Limit to tug's steering capability */
        tug_steer_cmd = clamp_value(tug_steer_cmd,
                                    -input->tug_max_steer,
                                    input->tug_max_steer);
    }
    
    /*
     * STEP 5: Calculate new tug heading based on steering
     */
    double tug_turn_radius;
    double new_tug_hdg = input->tug_hdg;
    
    if (fabs(input->tug_cur_steer) < 0.01) {
        /* Essentially straight - no heading change */
        tug_turn_radius = 1e10;
    } else if (fabs(input->tug_cur_steer) > 89.0) {
        /* Maximum steering - minimum radius */
        tug_turn_radius = input->tug_wheelbase * 0.01;
        if (input->tug_cur_steer < 0)
            tug_turn_radius = -tug_turn_radius;
    } else {
        /* Normal steering - calculate turn radius */
        tug_turn_radius = tan(DEG2RAD(90 - fabs(input->tug_cur_steer))) *
                          input->tug_wheelbase;
        if (input->tug_cur_steer < 0)
            tug_turn_radius = -tug_turn_radius;
    }
    
    if (fabs(tug_turn_radius) < 1e3 && fabs(input->tug_speed) > TOWBAR_MIN_SPEED) {
        double d_hdg = RAD2DEG(input->tug_speed / tug_turn_radius) * input->d_t;
        new_tug_hdg = normalize_heading(input->tug_hdg + d_hdg);
        
        /* Limit tug heading relative to aircraft backward direction */
        double acf_backward = normalize_heading(input->acf_hdg + 180.0);
        double rel_hdg = relative_heading(acf_backward, new_tug_hdg);
        
        if (rel_hdg > input->max_nw_steer)
            new_tug_hdg = normalize_heading(acf_backward + input->max_nw_steer);
        else if (rel_hdg < -input->max_nw_steer)
            new_tug_hdg = normalize_heading(acf_backward - input->max_nw_steer);
    }
    
    /*
     * STEP 6: Calculate tug position to maintain towbar connection
     */
    vect2_t new_tug_pos, hitch_pos;
    towbar_calculate_tug_position(state, nw_pos, input->acf_hdg, new_tug_hdg,
                                  &new_tug_pos, &hitch_pos);
    
    /*
     * STEP 7: Calculate aircraft heading correction
     * The aircraft heading changes as it is pushed/pulled via the towbar.
     */
    double tug_turn_rate = 0;
    if (fabs(tug_turn_radius) < 1e3 && fabs(input->tug_speed) > TOWBAR_MIN_SPEED) {
        tug_turn_rate = (input->tug_speed / (2 * M_PI * tug_turn_radius)) * 360.0;
    }
    
    /* Effective distance for heading change calculation */
    double effective_dist = state->towbar_length + state->hitch_z;
    double acf_hdg_delta = tug_turn_rate * input->d_t * 
                           (effective_dist / input->tug_wheelbase) *
                           params->hdg_damping;
    
    /* Apply amplification for physics engine */
    acf_hdg_delta *= TOWBAR_HDG_AMPLIFY;
    
    /*
     * STEP 8: Calculate animation values
     */
    double towbar_heading = hitch_angle;
    double towbar_pitch = 0;  /* Vertical angle - would need height data */
    
    /*
     * STEP 9: Update state for next frame
     */
    state->prev_tug_hdg = input->tug_hdg;
    state->prev_acf_hdg = input->acf_hdg;
    state->prev_hitch_angle = hitch_angle;
    state->hitch_pos = hitch_pos;
    state->nw_connection_pos = nw_pos;
    state->connected = B_TRUE;
    
    /*
     * STEP 10: Fill output structure
     */
    output->tug_steer_cmd = tug_steer_cmd;
    output->tug_new_pos = new_tug_pos;
    output->tug_new_hdg = new_tug_hdg;
    output->nw_steer = cur_nw_steer;
    output->acf_hdg_delta = acf_hdg_delta;
    output->towbar_heading = towbar_heading;
    output->towbar_pitch = towbar_pitch;
    output->hitch_angle = hitch_angle;
    output->connection_angle = connection_angle;
}

void
towbar_debug_log(const towbar_state_t *state,
                 const towbar_input_t *input,
                 const towbar_output_t *output)
{
    ASSERT(state != NULL);
    ASSERT(input != NULL);
    ASSERT(output != NULL);
    
    logMsg(BP_INFO_LOG "[TOWBAR] === Physics Debug ===");
    logMsg(BP_INFO_LOG "[TOWBAR] Towbar: length=%.2fm hitch_z=%.2fm",
           state->towbar_length, state->hitch_z);
    logMsg(BP_INFO_LOG "[TOWBAR] Input: acf_hdg=%.1f° tug_hdg=%.1f° tug_spd=%.2fm/s",
           input->acf_hdg, input->tug_hdg, input->tug_speed);
    logMsg(BP_INFO_LOG "[TOWBAR] Input: req_steer=%.1f° tug_cur_steer=%.1f°",
           input->req_steer, input->tug_cur_steer);
    logMsg(BP_INFO_LOG "[TOWBAR] Output: tug_steer_cmd=%.1f° nw_steer=%.1f°",
           output->tug_steer_cmd, output->nw_steer);
    logMsg(BP_INFO_LOG "[TOWBAR] Output: acf_hdg_delta=%.3f° hitch_angle=%.1f°",
           output->acf_hdg_delta, output->hitch_angle);
    logMsg(BP_INFO_LOG "[TOWBAR] Position: tug=(%.2f,%.2f) hitch=(%.2f,%.2f)",
           output->tug_new_pos.x, output->tug_new_pos.y,
           state->hitch_pos.x, state->hitch_pos.y);
}
