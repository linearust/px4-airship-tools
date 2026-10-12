// Identity harness (scratchpad only, never committed): drives the airship
// effectiveness through fixed scripts for several parameter sets and dumps
// every output as hex floats, so refactor commits can be cmp'd bit for bit.
#include <gtest/gtest.h>
#include "ActuatorEffectivenessAirship.hpp"
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
#include <cstdio>
#include <cstdlib>

using namespace matrix;

static void publishArmed(bool armed)
{
	static uORB::Publication<vehicle_status_s> pub{ORB_ID(vehicle_status)};
	vehicle_status_s vehicle_status{};
	vehicle_status.timestamp = hrt_absolute_time();
	vehicle_status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
	pub.publish(vehicle_status);
}

struct Surface { int32_t type; float trq_r, trq_p, trq_y, trim; };
struct ParamSet {
	const char *name; float tlmin, tlmax; int32_t grp, tail; float tlt_r, cs_k; int32_t cs_count; Surface cs[4]; float tail_min;
};

static void setParams(const ParamSet &p)
{
	param_control_autosave(false);
	param_reset_all();
	param_set(param_find("CA_AIRSHIP_TLMIN"), &p.tlmin);
	param_set(param_find("CA_AIRSHIP_TLMAX"), &p.tlmax);
	param_set(param_find("CA_AIRSHIP_GRP"), &p.grp);
	param_set(param_find("CA_AIRSHIP_TAIL"), &p.tail);
	param_set(param_find("CA_AIRSHIP_TLT_R"), &p.tlt_r);
	param_set(param_find("CA_AIRSHIP_CS_K"), &p.cs_k);
	param_set(param_find("CA_SV_CS_COUNT"), &p.cs_count);
	char name[17];

	for (int i = 0; i < p.cs_count; i++) {
		snprintf(name, sizeof(name), "CA_SV_CS%d_TYPE", i); param_set(param_find(name), &p.cs[i].type);
		snprintf(name, sizeof(name), "CA_SV_CS%d_TRQ_R", i); param_set(param_find(name), &p.cs[i].trq_r);
		snprintf(name, sizeof(name), "CA_SV_CS%d_TRQ_P", i); param_set(param_find(name), &p.cs[i].trq_p);
		snprintf(name, sizeof(name), "CA_SV_CS%d_TRQ_Y", i); param_set(param_find(name), &p.cs[i].trq_y);
		snprintf(name, sizeof(name), "CA_SV_CS%d_TRIM", i); param_set(param_find(name), &p.cs[i].trim);
	}
}

static uint32_t lcg_state = 12345u;
static float rnd()  // [-1.2, 1.2]
{
	lcg_state = lcg_state * 1664525u + 1013904223u;
	return ((lcg_state >> 8) / 16777216.f) * 2.4f - 1.2f;
}

TEST(AirshipIdentity, Dump)
{
	const Surface E = {3, 0.f, 0.5f, 0.f, 0.f}, R = {4, 0.f, 0.f, 0.5f, 0.f};
	const Surface E1 = {3, 0.f, 1.f, 0.f, 0.f}, R1 = {4, 0.f, 0.f, 1.f, 0.1f}, A1 = {1, 1.f, 0.f, 0.f, 0.f};
	const ParamSet sets[] = {
		{"2500", 0.f, 0.f, 0, 0, 0.f, 1.f, 4, {E, E, R, R}, 0.f},
		{"2504", -180.f, 180.f, 1, 0, 120.f, 1.f, 0, {}, 0.f},
		{"2507", -90.f, 90.f, 0, 1, 120.f, 1.f, 0, {}, -1.f},
		{"bench", -180.f, 180.f, 1, 0, 120.f, 0.5f, 4, {E, E, R, R}, 0.f},
		{"indep_tail_noslew", -180.f, 180.f, 1, 1, 0.f, 0.5f, 3, {E1, R1, A1}, -1.f},
		{"down_only", -180.f, 0.f, 0, 0, 90.f, 1.f, 0, {}, 0.f},
		{"up_only_tail", 0.f, 90.f, 1, 1, 60.f, 0.7f, 2, {E1, R1}, -1.f},
		// Wide asymmetric ranges: both rear ends point backward but realize
		// differently, so the rear-cone ranking, the end switch and the seam
		// switch all take their asymmetric paths (the three steerTarget
		// defects of 2026-09-11 only showed on ranges like these)
		{"asym_wide", -100.f, 99.f, 1, 0, 120.f, 1.f, 0, {}, 0.f},
		{"asym_wide_coll", -100.f, 99.f, 0, 1, 120.f, 1.f, 2, {E1, R1}, -1.f},
		{"asym_one_rear", -100.f, 60.f, 1, 0, 90.f, 1.f, 0, {}, 0.f},
		// Ends 180 deg apart, one just past vertical: no target is ever across
		// the seam, so only the rear-cone margin hold (117046d8a7) keeps a
		// straight-back demand
		{"asym_vertical", -88.f, 92.f, 1, 0, 120.f, 1.f, 0, {}, 0.f},
		{"asym_vertical_coll", -88.f, 92.f, 0, 1, 120.f, 1.f, 2, {E1, R1}, -1.f},
	};
	const float vectors[][4] = { // roll, pitch stick, yaw, thrust
		{0, 0, 0, 0}, {0, 0, 0, .5f}, {0, 0, 0, 1.f}, {0, 0, .5f, 0}, {0, 0, -.5f, 0}, {0, 0, .015f, 0},
		{.5f, 0, 0, 0}, {-.5f, 0, 0, .5f}, {0, .5f, 0, 0}, {0, -.5f, 0, .5f}, {0, 0, .5f, .5f}, {.3f, -.3f, .2f, .5f},
		{0, 0, 0, 0}, {0, 0, 0, -1.f}, {0, 0, 1.f, 0}, {0, 0, 0, 0},
	};
	const char *out_path = getenv("IDENTITY_OUT") ? getenv("IDENTITY_OUT") : "/tmp/identity_dump.txt";
	FILE *out = fopen(out_path, "w");
	ASSERT_NE(out, nullptr);
	const float dt = 0.02f;

	for (const ParamSet &p : sets) {
		setParams(p);
		lcg_state = 12345u;
		publishArmed(true);
		ActuatorEffectivenessAirship airship(nullptr);
		int step = 0;

		auto run = [&](const Vector<float, 6> &control_sp, int repeats) {
			ActuatorEffectiveness::Configuration configuration{};
			airship.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE);
			// The declared layout is motors, then control surfaces, then tilt
			// servos (getEffectivenessMatrix); the surface count is a parameter
			// and the tilt count follows the tilt range
			const int n_motor = p.tail ? 3 : 2;
			const int n_tilt = (p.tlmax - p.tlmin > 0.06f) ? (p.grp ? 2 : 1) : 0;
			const int first_surface = n_motor;
			const int first_tilt = n_motor + p.cs_count;
			ActuatorEffectiveness::ActuatorVector actuator_min{}; actuator_min.setAll(-1.f);

			for (int i = 0; i < n_motor; i++) { actuator_min(i) = 0.f; }

			if (p.tail) { actuator_min(2) = p.tail_min; }

			ActuatorEffectiveness::ActuatorVector actuator_max{}; actuator_max.setAll(1.f);

			for (int r = 0; r < repeats; r++) {
				ActuatorEffectiveness::ActuatorVector actuator_sp{};
				Vector3f surface_torque{};

				for (int i = 0; i < p.cs_count; i++) {
					const Vector3f trq{p.cs[i].trq_r, p.cs[i].trq_p, p.cs[i].trq_y};
					const float sp = trq(0) * control_sp(0) + trq(1) * control_sp(1) + trq(2) * control_sp(2);
					actuator_sp(first_surface + i) = sp;
					surface_torque += trq * (math::constrain(sp, -1.f, 1.f) - p.cs[i].trim);
				}

				control_allocator_status_s status{};

				for (int axis = 0; axis < 3; axis++) { status.unallocated_torque[axis] = control_sp(axis) - surface_torque(axis); }

				airship.allocateAuxilaryControls(dt, 0, actuator_sp);
				airship.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
				airship.getUnallocatedControl(0, status);
				fprintf(out, "%s %5d |", p.name, step++);

				for (int i = 0; i < n_motor; i++) { fprintf(out, " %a", (double)actuator_sp(i)); }

				fprintf(out, " S");

				for (int i = 0; i < p.cs_count; i++) { fprintf(out, " %a", (double)actuator_sp(first_surface + i)); }

				fprintf(out, " T");

				for (int i = 0; i < n_tilt; i++) { fprintf(out, " %a", (double)actuator_sp(first_tilt + i)); }

				fprintf(out, " |");

				for (int i = 0; i < 3; i++) { fprintf(out, " %a", (double)status.unallocated_torque[i]); }

				for (int i = 0; i < 3; i++) { fprintf(out, " %a", (double)status.unallocated_thrust[i]); }

				fprintf(out, "\n");
			}
		};

		for (const float *v : vectors) {
			Vector<float, 6> control_sp{};
			control_sp(0) = v[0]; control_sp(1) = -v[1]; control_sp(2) = v[2]; control_sp(3) = v[3]; control_sp(5) = v[1];
			run(control_sp, 200);
		}

		for (int i = 0; i < 2000; i++) {
			if (i % 300 == 0) { publishArmed((i / 300) % 2 == 0); }

			Vector<float, 6> control_sp{};

			for (int a = 0; a < 6; a++) { control_sp(a) = rnd(); }

			run(control_sp, 1 + (i % 3));
		}

		publishArmed(false);
		Vector<float, 6> idle{};
		run(idle, 200);
		publishArmed(true);
	}

	fclose(out);
}
