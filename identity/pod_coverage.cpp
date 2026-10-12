
#include "AirshipPod.hpp"
#include "pod_coverage.hpp"
#include <cstdio>
#include <cmath>
Cov cov;
using namespace matrix;
static uint32_t lcg_state = 12345u;
static float rnd(){ lcg_state = lcg_state * 1664525u + 1013904223u; return ((lcg_state >> 8) / 16777216.f) * 2.4f - 1.2f; }
struct Set { const char *name; float tlmin, tlmax; int grp; float tlt_r; };
int main(){
	const Set sets[] = {
		{"2504", -180.f, 180.f, 1, 120.f}, {"down_only", -180.f, 0.f, 0, 90.f}, {"up_only_tail", 0.f, 90.f, 1, 60.f},
		{"asym_wide", -100.f, 99.f, 1, 120.f}, {"asym_wide_coll", -100.f, 99.f, 0, 120.f}, {"asym_one_rear", -100.f, 60.f, 1, 90.f},
		{"asym_wide_noslew", -100.f, 99.f, 1, 0.f},
		{"asym_vertical", -88.f, 92.f, 1, 120.f}, {"asym_vertical_coll", -88.f, 92.f, 0, 120.f},
	};
	const float vectors[][4] = { {0,0,0,0},{0,0,0,.5f},{0,0,0,1.f},{0,0,.5f,0},{0,0,-.5f,0},{0,0,.015f,0},
		{.5f,0,0,0},{-.5f,0,0,.5f},{0,.5f,0,0},{0,-.5f,0,.5f},{0,0,.5f,.5f},{.3f,-.3f,.2f,.5f},
		{0,0,0,0},{0,0,0,-1.f},{0,0,1.f,0},{0,0,0,0} };
	const float dt = 0.02f;
	printf("%-19s %6s %5s %5s %5s %5s | %6s %5s %5s | %5s %5s | %5s %5s\n","set","total","rear","rpick","rmove","rtie","outrng","ohold","oswit","seamc","seamh","conep","coneh");
	for (const Set &s : sets) {
		cov.reset(); lcg_state = 12345u;
		AirshipPod pods[2]; bool armed = true;
		for (AirshipPod &p : pods) { p.setTiltRange(s.tlmin * (float)M_PI / 180.f, s.tlmax * (float)M_PI / 180.f); p.setTiltSlewRate(s.tlt_r * (float)M_PI / 180.f); }
		auto run = [&](const Vector<float,6> &c, int repeats){
			for (int r = 0; r < repeats; r++) {
				const Vector2f common{c(3), -c(5)};
				const Vector2f diff = s.grp ? Vector2f{c(2), c(0)} : Vector2f{};
				const Vector2f d[2] = {common - diff, common + diff};
				for (int i = 0; i < 2; i++) { if (armed) pods[i].steer(d[i], dt); else pods[i].park(dt); }
				// servo write/read-back round trip as writeTiltServos() does
				const int n = pods[0].canTilt() ? (s.grp ? 2 : 1) : 0;
				for (int i = 0; i < n; i++) { float sp = pods[i].servoSetpoint(); sp = sp < -1.f ? -1.f : (sp > 1.f ? 1.f : sp); pods[i].setServoSetpoint(sp); }
				if (!s.grp && n) { pods[1].setServoSetpoint(pods[0].servoSetpoint()); }
			}
		};
		for (const float *v : vectors) { Vector<float,6> c{}; c(0)=v[0]; c(1)=-v[1]; c(2)=v[2]; c(3)=v[3]; c(5)=v[1]; run(c, 200); }
		for (int i = 0; i < 2000; i++) { if (i % 300 == 0) armed = (i / 300) % 2 == 0; Vector<float,6> c{}; for (int a = 0; a < 6; a++) c(a) = rnd(); run(c, 1 + (i % 3)); }
		armed = false; Vector<float,6> idle{}; run(idle, 200); armed = true;
		printf("%-19s %6ld %5ld %5ld %5ld %5ld | %6ld %5ld %5ld | %5ld %5ld | %5ld %5ld\n", s.name, cov.total, cov.rear, cov.rear_pick, cov.rear_pick_moves, cov.rear_tie, cov.outrange, cov.out_hold, cov.out_switch, cov.seam_checked, cov.seam_held, cov.cone_priced, cov.cone_hold);
	}
}
