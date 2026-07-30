# Hardware data archive manifest

The raw hardware telemetry (CSV time series and settle logs) lives
in the operator's private data archive OUTSIDE this repository;
paths below are relative to that archive's root. The small
`.dwells.json` sidecars — the tuning records, dwell verdicts, and
demodulated estimates the documentation quotes — are tracked
in-repo under `data/` (one per starred file below). Checksums are
SHA-256 of each file as archived on 2026-07-28; a citation in the
project records is verifiable by checking the named file against
its hash. Project records and code comments cite these datasets by
bare filename (e.g. `track8.csv`, `pass1.csv`, `p1_j1_survey_r2.csv`)
— every such name resolves to a row in the tables below.

## `m8-tracking/` — M8 torque-tracking runs (2026-07-21)

| file | role | sha256 |
|---|---|---|
| `track.csv` | M8 bring-up tracking run (development) | `77103401ff5b98748220e9dcac06d678efa5f2fa75e5806f28552f17a0887534` |
| `track2.csv` | M8 bring-up tracking run (development) | `03d0a1fb0f8315bcfff696a8058f1e65f8b311e4ab4c04aca1b38778f84950fe` |
| `track3.csv` | M8 bring-up tracking run (development) | `b781db3606148f5710e0d732ee1b238cb2f145cf4f395fa6c797b0b30463ebb9` |
| `track4.csv` | M8 bring-up tracking run (development) | `0cebc358e014f4594722ac91163c9e422ded67c57570a0abc70f642ec2a1c214` |
| `track5.csv` | M8 bring-up tracking run (development) | `a156b5a505b79ef41763f7c743f48ab4a27fb82888ea6845c4f72413df2d54c3` |
| `track6.csv` | M8 bring-up tracking run (development) | `73ff4d00d12b2d029fa4ceb1e26eb895f6ab62aeef98ea2937b043ff4960b5d1` |
| `track7.csv` | Run 7: full-scale ~4-5 Hz oscillation evidence; P2 anchor source; cited in theory/computed-torque.md; input for the documented reopening path | `7daa40b87a44e5115643a3d5d5c0a4b75eefb5decd223bacc1aec328b68e9db6` |
| `track8.csv` | Run 8: full-scale oscillation evidence (manual power cut); cited in theory/computed-torque.md | `b64d079636c8c5803e60b5556e7e1d5f27e6580ab35bd5741d570f93c4bdc7cf` |

## `pass1/` — Pass-1 instrumentation validation (2026-07-23)

| file | role | sha256 |
|---|---|---|
| `pass1.csv` | Pass-1 instrumentation validation at scale 0.5; cited in REMEDIATION_PLAN.md status; P1 anchor source | `a33b8efcde6acd3d7313f5eb190fb84768a8d38d3cd4a53e70ee287ffca0177c` |
| `pass1.csv.settle` | Settle-gate log of the pass-1 validation run | `b7caa9b04f251372ec65f1dc210cc986cce9d5ee3cf1d1200144e2b790575df9` |

## `p1-holds/` — P1 hold diagnostics and qualification (2026-07-27)

| file | role | sha256 |
|---|---|---|
| `p1_hold_s050.csv` \* | Hold-diagnostic session 1 at joint-0 scale 0.5 (log overwritten twice before unique naming; final run only) | `3a5fbd091d8313992077e6421d23de2710d4e56d6a57bc2ec721125e967c9691` |
| `p1_hold_s050.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_s050.csv.dwells.json`) | `edb4919cff42eb265fe5a2ab1f41cadc3c5e135dd65ce846cf3c5c86e31d94a5` |
| `p1_hold_s050_r1.csv` \* | Hold session 2 run 1 (live integrator): joint-2 integrator-stiction hunting | `87a01a57be2cc5c3708a0adfbf7e9c7deee0631c679258f9aebd07352702b197` |
| `p1_hold_s050_r1.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_s050_r1.csv.dwells.json`) | `89b12f22a471cf10c28cb4aa2ca0e79076e11421bf0d3abfb96889fa7430b2ec` |
| `p1_hold_s050_r2.csv` \* | Hold session 2 run 2 (live integrator): hunting recurrence | `8e91f48a431734531684549f557d8f0666b930f4b78c5b303d51b9d75a80909d` |
| `p1_hold_s050_r2.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_s050_r2.csv.dwells.json`) | `7b058704c9d08dec10e951baebaf8f84be1895b2663fa491ebf1630fc8187822` |
| `p1_hold_s050_r3.csv` \* | Hold session 2 run 3 (live integrator): hunting recurrence | `fceefa570b83f36085637577ba40dbfd3cbb9ff1cb61bf7305fecf80f52175e6` |
| `p1_hold_s050_r3.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_s050_r3.csv.dwells.json`) | `61cdf4aef3d01e68dfda9f99c6cf08d107a01b6857932dd8ab5e3598c107cc7e` |
| `p1_hold_s050_fz_r1.csv` \* | Frozen-integral qualifying hold 1/3 PASSED (30 s) (sidecar predates bias recording — analyzer refuses it by design) | `309460038b22681c62985f20e0bcc8888c71fe7cde6411b973feb64ab7f6d7e4` |
| `p1_hold_s050_fz_r1.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_s050_fz_r1.csv.dwells.json`) | `59dc021ef89bdd11c7ff056c43c33099b74f84df38dfddfe4908d1921715acc0` |
| `p1_hold_s050_fz_r2.csv` \* | Frozen-integral qualifying hold 2/3 PASSED (30 s) (sidecar predates bias recording — analyzer refuses it by design) | `a8e9319369a269f7507de86aedd2a3318fdad4bbec8c9f4a74e7c6d21c6ec4be` |
| `p1_hold_s050_fz_r2.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_s050_fz_r2.csv.dwells.json`) | `00031606dc4a784d158dccf9700c43b538a260ece3fee1527974be64c5962677` |
| `p1_hold_s050_fz_r3.csv` \* | Frozen-integral qualifying hold 3/3 PASSED (30 s) (sidecar predates bias recording — analyzer refuses it by design) | `7d212604bff0c2a6e2d5d67afd9bc87614d628c2f77a6830b10d282fbcd7b93f` |
| `p1_hold_s050_fz_r3.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_s050_fz_r3.csv.dwells.json`) | `4ac40feb740937f1c55d272078c8af31409abb3a482a11834e7c3d5e0beedcfb` |
| `p1_hold_prod_r1.csv` \* | Production-path equivalence hold PASSED (30/30 s) | `8902128f52e8dc9e7ffbce555459f765030dfdf5df6fdf8f9e87cd88000838f9` |
| `p1_hold_prod_r1.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_prod_r1.csv.dwells.json`) | `d18f28dbd79dcf45a867eea4aa27ab58441f646e6be4821eba45ba1de360d0bd` |
| `p1_hold_prod_r2.csv` \* | Re-qualification hold under per-joint latching (integral_policy 2) PASSED | `1f549c5ea6ae3aa70f7074b45f1a7aebf5892d28937b1d3970fd6d7d22e68dde` |
| `p1_hold_prod_r2.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_hold_prod_r2.csv.dwells.json`) | `9d2814e52a4ea0e2039fc9cdd8f83481b8cb415663c7f1a3e45ff02d296ddd07` |

## `p1-ident/` — P1 joint-1 identification and amplitude pilots (2026-07-27)

| file | role | sha256 |
|---|---|---|
| `p1_j1_survey.csv` \* | FAILED first production survey (capture timeout, joint-3 winding) - NO probe data, never analyze | `c77e70e9706ec6274fcaa44d6f4feaf4c8433c3afa00a4e47e26641f6fe11cc9` |
| `p1_j1_survey.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_j1_survey.csv.dwells.json`) | `6c448b7599de088e50cf68e98b17e91844666435b63850970ae4400d3e166c82` |
| `p1_j1_survey.csv.settle` | Settle log of the failed survey attempt | `e62d806a40f559a817b34fdb18376ab9dbcea53473e9d25242cb818750a043fe` |
| `p1_j1_survey_r2.csv` \* | The null survey: joint 1 tick-frozen through 13/16 windows; cited in IDENTIFICATION_PLAN.md Closure | `0a905c086ce312f549f2dcaa0a9796eedbc30f2f65969982144eed830dd44a95` |
| `p1_j1_survey_r2.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_j1_survey_r2.csv.dwells.json`) | `60f7d80b447417ee5065dd0c3c1ca14ad3cbc1aff06d78a9bb6d383ee9a6ecd6` |
| `p1_j1_amp020.csv` \* | Amplitude pilot 0.20 Nm at 4.5 Hz (null); cited in the Closure | `054214d7955e597d2e3b46aecf4c7383a09912c6382dd6f21c13b36a96d1ea78` |
| `p1_j1_amp020.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_j1_amp020.csv.dwells.json`) | `5bb21d33cdad779b56317c43c3613f5cde62bac259a35ab0ab5c5e6e29104f5b` |
| `p1_j1_amp025.csv` \* | Amplitude pilot 0.25 Nm at 4.5 Hz (null); cited in the Closure | `36a9c554c6f3db77e233fa78f759a2b6367f76faab577f97b1c52484b3d2deaf` |
| `p1_j1_amp025.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_j1_amp025.csv.dwells.json`) | `5ead36919a419d8fbc08628ee3c92e0751e872c44fee7e50a6a69a189fa9a2bb` |
| `p1_j1_amp030.csv` \* | Amplitude pilot 0.30 Nm (hard cap) at 4.5 Hz (null) - the stop condition; cited in the Closure | `74cdf8e9a6bbcd5be17a55ccc0c10795b6f89e29334526bcc5189335e10964e1` |
| `p1_j1_amp030.csv.dwells.json` | sidecar of the above (also tracked at `data/p1_j1_amp030.csv.dwells.json`) | `b004cbcfc73111bc38f5e84efb740b3d8c8c9c78f7d9f98e7204b03a1d371a11` |

\* the run's `.dwells.json` sidecar is tracked in-repo under `data/`.

## `incidents-2026-07/` — parked-controller incident telemetry

Evidence of the two incidents that parked the current-mode
controllers; cited by `docs/GRAVITY_CALIBRATION_PLAN.md`,
`docs/HARDWARE_BRINGUP.md`, and the theory chapters. `track9.csv` was
logged as `track.csv` and renamed at archive time (the bare name
collides with the M8 entry above; the numbering continues runs 1–8).

| file | role | sha256 |
|---|---|---|
| `track9.csv` | 2026-07-28 x7_track scale-0.5 attempt: settle-phase pan anti-damping at 4.33 Hz, manual power cut; the run's (nearly empty) tracking log | `9969dc95a14f70b97a97fa89657623e5bc6ad12678c298259be4c317f311a7ca` |
| `track9.csv.settle` | settle-phase telemetry of the above: the 4.33 Hz growth 50 → 350 mrad p-p; basis of the settle-incident investigation | `ab599c6dbaf1e3ca5afa74b6bb055e7adae3ddfebf17c96ca6a2c87383a4d641` |
| `float1.csv` | 2026-07-29 FAILED x7_float test: untouched arm climbed upright (peak ~2.37 rad/s); basis of GRAVITY_CALIBRATION_PLAN.md's M-GC0 addendum | `6e0f72a74adaf5db129053a37ceb1724e60b738181ff1c23b01156012247b6fe` |

## `gravity-calibration/` — M-GC3 calibrated float tests (2026-07-30)

Evidence of the gravity-calibration remediation's hardware milestone
(`docs/GRAVITY_CALIBRATION_PLAN.md`, M-GC3): the vendor-scale
configuration, marker-first release, displacement-bounded acceptance.

| file | role | sha256 |
|---|---|---|
| `float2.csv` | M-GC3 acceptance run: PASSED all four preregistered conditions (peak speed 0.024 rad/s, displacement 0.0123 rad, zero gates/clamps, final-second drift ≤ 0.0015 rad); marker 4.080 s, window complete | `de1f1a3b333b26534b5617d5494a9c194e6e9c4555e32fcbeb81c7b02141fbf1` |
| `float3.csv` | M-GC3 back-drive feel-check run (operator back-driving during the window — NOT acceptance evidence): reported distinctly lighter than the 2026-07-29 failed run; zero gates/clamps, 503/503 accepted | `f09057f289066be6d60a9154c97f71ba141e7daccd8b993670ba0df3da7726fd` |
