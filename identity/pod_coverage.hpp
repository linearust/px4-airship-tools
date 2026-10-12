#pragma once
struct Cov { long total{}, rear{}, rear_pick{}, rear_pick_moves{}, rear_tie{}, outrange{}, out_hold{}, out_switch{}, seam_checked{}, seam_held{}, cone_priced{}, cone_hold{}; void reset(){*this=Cov{};} };
extern Cov cov;
