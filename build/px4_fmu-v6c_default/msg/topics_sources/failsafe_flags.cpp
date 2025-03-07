/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/vinay/Downloads/PX4-Autopilot/msg/FailsafeFlags.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_failsafe_flags_fields[] = "\x89 timestamp;\x88 mode_req_angular_velocity;\x88 mode_req_attitude;\x88 mode_req_local_alt;\x88 mode_req_local_position;\x88 mode_req_local_position_relaxed;\x88 mode_req_global_position;\x88 mode_req_mission;\x88 mode_req_offboard_signal;\x88 mode_req_home_position;\x88 mode_req_wind_and_flight_time_compliance;\x88 mode_req_prevent_arming;\x88 mode_req_manual_control;\x88 mode_req_other;\x8c angular_velocity_invalid;\x8c attitude_invalid;\x8c local_altitude_invalid;\x8c local_position_invalid;\x8c local_position_invalid_relaxed;\x8c local_velocity_invalid;\x8c global_position_invalid;\x8c auto_mission_missing;\x8c offboard_control_signal_lost;\x8c home_position_invalid;\x8c manual_control_signal_lost;\x8c gcs_connection_lost;\x86 battery_warning;\x8c battery_low_remaining_time;\x8c battery_unhealthy;\x8c primary_geofence_breached;\x8c mission_failure;\x8c vtol_fixed_wing_system_failure;\x8c wind_limit_exceeded;\x8c flight_time_limit_exceeded;\x8c local_position_accuracy_low;\x8c fd_critical_failure;\x8c fd_esc_arming_failure;\x8c fd_imbalanced_prop;\x8c fd_motor_failure;\x86[3] _padding0;";

ORB_DEFINE(failsafe_flags, struct failsafe_flags_s, 85, __orb_failsafe_flags_fields, static_cast<uint8_t>(ORB_ID::failsafe_flags));


void print_message(const orb_metadata *meta, const failsafe_flags_s& message)
{
	if (sizeof(message) != meta->o_size) {
		printf("unexpected message size for %s: %zu != %i\n", meta->o_name, sizeof(message), meta->o_size);
		return;
	}
	orb_print_message_internal(meta, &message, true);
}
