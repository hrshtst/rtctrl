# Hardware data archive manifest

The raw hardware telemetry (CSV time series and settle logs) lives
in the operator's private data archive OUTSIDE this repository;
paths below are relative to that archive's root. The small
`.dwells.json` sidecars — the tuning records, dwell verdicts, and
demodulated estimates the documentation quotes — are tracked
in-repo under `data/` (one per starred file below). Checksums are
SHA-256 at the archival date stated by each section; the older tables
below were archived on 2026-07-28. The control-rate sections (`e0/`,
`e1_prod/`, `e1_cal/`, `r4_bounded/`; checksums taken 2026-11-04)
are kept on a separate volume of the same private archive; their paths are
relative to that volume's project folder. A citation in the project records
is verifiable by checking the named file against its hash. Project
records and code comments cite these datasets by
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
| `pass1.csv` | Pass-1 instrumentation validation at scale 0.5; cited in history.md (pass-1 evidence); P1 anchor source | `a33b8efcde6acd3d7313f5eb190fb84768a8d38d3cd4a53e70ee287ffca0177c` |
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
| `p1_j1_survey_r2.csv` \* | The null survey: joint 1 tick-frozen through 13/16 windows; cited in history.md (identification closure) | `0a905c086ce312f549f2dcaa0a9796eedbc30f2f65969982144eed830dd44a95` |
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
controllers; cited by `docs/records/history.md`,
`docs/hardware/bringup.md`, and the theory chapters. `track9.csv` was
logged as `track.csv` and renamed at archive time (the bare name
collides with the M8 entry above; the numbering continues runs 1–8).

| file | role | sha256 |
|---|---|---|
| `track9.csv` | 2026-07-28 x7_track scale-0.5 attempt: settle-phase pan anti-damping at 4.33 Hz, manual power cut; the run's (nearly empty) tracking log | `9969dc95a14f70b97a97fa89657623e5bc6ad12678c298259be4c317f311a7ca` |
| `track9.csv.settle` | settle-phase telemetry of the above: the 4.33 Hz growth 50 → 350 mrad p-p; basis of the settle-incident investigation | `ab599c6dbaf1e3ca5afa74b6bb055e7adae3ddfebf17c96ca6a2c87383a4d641` |
| `float1.csv` | 2026-07-29 FAILED x7_float test: untouched arm climbed upright (peak ~2.37 rad/s); basis of the M-GC0 addendum (history.md, gravity calibration) | `6e0f72a74adaf5db129053a37ceb1724e60b738181ff1c23b01156012247b6fe` |

## `gravity-calibration/` — M-GC3 calibrated float tests (2026-07-30)

Evidence of the gravity-calibration remediation's hardware milestone
(`docs/records/history.md`, M-GC3): the vendor-scale
configuration, marker-first release, displacement-bounded acceptance.

| file | role | sha256 |
|---|---|---|
| `float2.csv` | M-GC3 acceptance run: PASSED all four preregistered conditions (peak speed 0.024 rad/s, displacement 0.0123 rad, zero gates/clamps, final-second drift ≤ 0.0015 rad); marker 4.080 s, window complete | `de1f1a3b333b26534b5617d5494a9c194e6e9c4555e32fcbeb81c7b02141fbf1` |
| `float3.csv` | M-GC3 back-drive feel-check run (operator back-driving during the window — NOT acceptance evidence): reported distinctly lighter than the 2026-07-29 failed run; zero gates/clamps, 503/503 accepted | `f09057f289066be6d60a9154c97f71ba141e7daccd8b993670ba0df3da7726fd` |
| `feel_j1_diag1.csv` | Reviewer-authorized gate-free j1 diagnostic session (2026-07-31, v2 instrumented log: event marks + raw counts; NOT acceptance evidence): VALID — `--feel --gate-free` passes, 6001 rows, 9 operator event marks, zero clamps/gates on every axis. Findings: host goal smooth (max 4 counts/cycle over the whole run); static current delivery clean (stationary error ≤ 7 counts at goals up to ~300 counts); all 93 present-current steps > 30 counts (max 73 ≈ 0.47 Nm nominal) occurred DURING motion, none stationary — breakaway/reversal transients; the two most violent in-window transients coincide with the 31.15 s / 34.93 s notch marks | `cd7a2ddac280e2b313213e6c6fc789849e4930c51c98d9398494a755ecf2ba6f` |
| `feel_j0.csv` … `feel_j6.csv` | M-GC3 per-joint back-drive sessions (feel-check mode, one target joint each, bidirectional coverage log-confirmed on every target; NOT acceptance evidence). Reviewer disposition (2026-07-30, supersedes the preliminary stiction attribution): subjective feel FAILED on j1 (random notchiness; power-off check clean, so torque-ENABLED — suspects XM540 current regulation / load-dependent drivetrain friction / cross-axis j3 gate discontinuities (18 gated cycles in the j1 session, ≈1.01 Nm zeroed ~33.6 s while j1 was stationary — uncorrelatable, no event marks), NOT the gravity model: the trajectory replay agrees within ≈5 % near the high-command extreme; lower-load trajectories show amplitude ratios differing ~10–12 % with small absolute differences) and inconclusive/failed on j4 (hand-guided positive tendency; likely compensation-induced — replay shows rtctrl at ~1.4× the vendor model's twist amplitude, within 1–3 goal counts); j3/j6 pass (symmetric rigidity). Zero clamps, all submissions accepted; position gates by axis: j0 92 / j3 18 (j1 session) / j4 853 / j6 91 cycles (endpoint dwell) | j0 `290b95c786003e7a0c5666c65749a85d9a992d16457dfca16a02ee118f3c5ec6` j1 `d33b607c68c3a8eb2a2c4b354e69ffb56c43ddf674725aca8b92143dd7aedda8` j2 `b01a7299cf77fed9710097175530d3275afad39eacbb7d17d244e98f9bf45e73` j3 `2a5901b2dcad29499ae49d809112bb8977dc22c56b83ef4d387d4cefc1219316` j4 `b8af89f405ec5e4d668a90da14b6a1c2614ddaa035ff49667d454253712b6327` j5 `1ae7c471a86455d90937e864a6c963313147ddc66e60959919fdde23cb37fb26` j6 `3902dbcc17fc7c9b8aacb2577e2bc143da767ddbb8c660071ec1277407a565b6` |

## `follow-cbp-gain-sweep-20260827/` — servo-side gain sweep (2026-08-27)

Complete x7_follow simulation and hardware bundles for the position versus
current-based position comparison and the subsequent CBP gain sweep. The
archive also contains motor dumps, derived comparison plots, all intermediate
TOML configurations, and patches recovered from the reflog after the accidental
hard reset. The selected tuning and rejected alternatives are summarized in
[history.md](history.md#current-based-position-gain-decision-2026-08-27).

| file | role | sha256 |
|---|---|---|
| `SHA256SUMS` | Archive-wide inventory for 258 files (138 MiB); verified after copying from `backups/` and `/tmp` | `15e9749ec614e2e6d64e58b919b6e3790ca2b873cf38812568040968db7d83d4` |
| `README.md` | Decision summary, run inventory, source locations, and recovery notes | `bb15f9b844090abb1c8a86f1fbf1240ae21b0a1b00fd58aba171a38b8a32b8d5` |
| `follow-cbp-75pct-d-id5-p900-20260827-181857/manifest.toml` | First run of the selected tuning | `7e486653a024f8ca2becea728e757a744e7055ae937c632a2d4d55dffbded4cf` |
| `follow-cbp-75pct-d-id5-p900-20260827-182554/manifest.toml` | Selected-tuning repeat 1 | `74fdee3492f5084db5023725e7fac9aaa142c2a576e03d98a5189dcae12d0b61` |
| `follow-cbp-75pct-d-id5-p900-20260827-182636/manifest.toml` | Selected-tuning repeat 2 | `cc49732bd514ea66009a66290885efa8017b636510df74f39c5da000f1cda856` |
| `follow-cbp-25pct-d-20260827-192302/manifest.toml` | Rejected 25% D test | `0bd4e44f76fd7d9d26f10968bab278f638b2b8f1845da183ab5806816889072e` |

## `teaching-playback-20260828-143834/` — manual teaching and playback (2026-08-28)

Complete bundles for torque-off and calibrated gravity-compensation teaching,
the position/CBP dynamics simulations, and the corresponding hardware
comparison. The archived playback bundles use the torque-off taught trajectory;
the gravity-compensation teaching bundle retains its separately confirmed
trajectory and full-session log. Derived planner, simulation, and hardware
comparison figures and CSV summaries live under `analysis/`.

| file | role | sha256 |
|---|---|---|
| `teaching/manifest.toml` | Successful torque-off teaching: 1,924 raw samples, 1,152 ZVS frames, 11.520 s | `15a189f03964c7d68f4efcefb8d315b29970cac2974cda0378c3b09d0c6b43a6` |
| `teaching-gc/manifest.toml` | Successful gravity-compensation teaching: 2,514 raw samples, 1,216 ZVS frames, 12.160 s | `43b6b788dd4238a5c2337b60164abcc765d9a967be37e53fb12b9ad9d1733b06` |
| `simulation/position/manifest.toml` | Successful position-mode dynamics playback | `4961c4617dbdbec4d84af6fbaa8348101a6826b3663b429b7be5d28bc03d8483` |
| `simulation/cbp/manifest.toml` | Successful CBP dynamics playback with the selected tuned parameter input | `e54d6be7ded4c9d4d5bdc5638b26f3aa3a43e654ca49a46ad947648e3cc3001f` |
| `position/manifest.toml` | Successful position-mode hardware playback | `2e4f68d7f047a34ce92340b3b80db2c5846edbdd705bf800c1af29367aaba475` |
| `cbp/manifest.toml` | Successful tuned-CBP hardware playback | `74c2475f3e976eaf96a3270f5983eccff0facdfdc6200be7c0a406d263026f92` |
| `analysis/hardware-comparison.csv` | Per-joint hardware RMS and peak error comparison | `247abc0a34b612e3e5e778df848b76c6af7189ac4f9b3c2183335eab78f999ab` |

## `e0/` — control-rate E0 inventory (2026-11-03)

The torque-off inventory and the servo parameter baseline taken before E1;
cited in [history.md](history.md#control-rate-exploration-e0-and-e1-2026-11-03).
Archived 2026-11-04 from the copies kept untracked at the repository root.

| file | role | sha256 |
|---|---|---|
| `rate-e0-inventory-20261103.toml` | E0 inventory (`x7_bus_timing --inventory`): servo registers, adapter, latency timer, kernel and governors | `0fac4473549fbd3db8a62c63eb074bf799dcb3e9333bcebb80b0297bd752a5c6` |
| `x7-baseline-20261103.toml` | Servo parameter baseline (`dxl_inspect dump-params`, raw units) from the same session; records Return Delay Time 0 on every servo (B1) | `074077e022e3310fc2eb148fe9f446458a70a6658ac47010645ca0c294960ebd` |

## `e1_prod/` — control-rate E1 production traffic, legacy read path (2026-11-03)

33 torque-off probe runs of 60 s (`tools/e1_prod.sh` at `dba323d`), three files
each; the climbs stopped at the first clearly failing rate, so there is no
1000 Hz run and no loaded 500 Hz run. Exploration only, never acceptance
evidence; cited in [history.md](history.md#control-rate-exploration-e0-and-e1-2026-11-03).

| file | role | sha256 |
|---|---|---|
| `rate-e1-prod-cbp-100hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `58e55c57ffceef676a0d563a8800e2b5c7814d934d71f44eebe27c25773d4f03` |
| `rate-e1-prod-cbp-100hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `54d99690901eaa4a6524c82680e2b91a594a2ab17b43a00749f72cc71d13347e` |
| `rate-e1-prod-cbp-100hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `8ad66b8d2f17c5545324ff4e73ef94083f7e2c7ccedfabb32e8fbf537f2808e0` |
| `rate-e1-prod-cbp-100hz-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `b78e71883938f6ca60c1549a6c0febc60b91bafe911130bd18d3ba48303f35d3` |
| `rate-e1-prod-cbp-100hz-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `10f313cb5be9619854ef2b118af5cebe75205f58fe7005939c3c98fe4115a4aa` |
| `rate-e1-prod-cbp-100hz-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `7a358b86e2d96d05c549fd1ba569fd63edf725d3f5fb0f0f6e358e2bfe7ef06c` |
| `rate-e1-prod-cbp-100hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `fd6cb51fae5f2ea09cbe9f021bfc2dbc49dda4cc83986975e23b4d0435a06d74` |
| `rate-e1-prod-cbp-100hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `c3cf685831669f66039d14414da2e9dd3b8e62cea79c1ae40a02e3fff6cc3349` |
| `rate-e1-prod-cbp-100hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `c1b6c294c528f5383b32c896660d6425ed0e823c961dc7939150d9529b8ecc76` |
| `rate-e1-prod-cbp-100hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `c71afd97514c2dfbaacc8133a1d0611e5ae9b5fe92a706efc52b125bfe310a08` |
| `rate-e1-prod-cbp-100hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `e6fed7e52803c4e934c4ced5aeb35c4fe8e89843fa1869ef5b213f3b9f03ee7f` |
| `rate-e1-prod-cbp-100hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `b1c074b7c30e8219d422b0fc4b93c4dee4cd6a098d665fc1c45d632ae722a8d5` |
| `rate-e1-prod-cbp-200hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `8a8fbe57256f1ccd3a242739359597c010e020d2c43a5c4495d379b71db7949f` |
| `rate-e1-prod-cbp-200hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `44701ff56e484f119180b1e4a58abbf01d3132dab39e5654969d7c6ce66a8168` |
| `rate-e1-prod-cbp-200hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `3f0afe2bba7603ee45d6d9251cf7c9007bfbfa25a4228fbb66edeb2544ea7d5d` |
| `rate-e1-prod-cbp-200hz-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `f0bac077455f61744f9a8082894f9205234cc2a1af1f133250a31c428e32324a` |
| `rate-e1-prod-cbp-200hz-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `87655640b669f7d205455c892071a04df069f615c670b7a6ba024cbf40b59243` |
| `rate-e1-prod-cbp-200hz-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `cb75268b6d0500994faa99095cdf739447f8ceec1d47580cbfdba17a0ade957b` |
| `rate-e1-prod-cbp-200hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `053aa593e4bea35a25477c70d9ae48dc26e5cb0e0e4e561df481d49af0a1e557` |
| `rate-e1-prod-cbp-200hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `b48046646069698f1d88229a21aafd6535511bc28db5ce3a5c5bfb5e339157cc` |
| `rate-e1-prod-cbp-200hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `2922a58535c26e4f061ca63bbca62a72c4260d389ec63e971483031794201988` |
| `rate-e1-prod-cbp-200hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `6f98709604a4d73689af63a06d87fa8819d8df1032b6d62201d2e379dd9f00ed` |
| `rate-e1-prod-cbp-200hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `580f248abe267b8245270c2c14e098715b661aa3976d776be2e005a9b185061f` |
| `rate-e1-prod-cbp-200hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `65bc0d9ad30bfe3b47cfb7465433ebf49b5e19ff6fce7f4fabfe9631f7991842` |
| `rate-e1-prod-cbp-500hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `5f8934a28807d8f253dfe975d92785fdeb708ac3bd35e3d814495843fee66c07` |
| `rate-e1-prod-cbp-500hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `cc5511824fe0e122167a4c77717cae60768715dfbe2e36dc75b0865635d4b9a1` |
| `rate-e1-prod-cbp-500hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `4a4765039df5d680a4813aaf62d3336980453907eff967c92f77bbee4be0183f` |
| `rate-e1-prod-cbp-500hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `bca139160acd23370e9ca680bd3fe5f38c4bdb16b2453dd3c15248397f839299` |
| `rate-e1-prod-cbp-500hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `33b2358e7ad5420159648847b34e3137a54f71b56c7e958648a23b6f04e56e78` |
| `rate-e1-prod-cbp-500hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `2429f5a62269df2c1be6b564a386dace92bcd0f00b62acdda94ff259859688be` |
| `rate-e1-prod-cbp-500hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `1cbd0749dcc1ca249ba9dcc09d05824dc221aece840f716153878db71a1ea61e` |
| `rate-e1-prod-cbp-500hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `c3655844185714c87db896530bb1123e3d87a0d0fa67cb1273ebf6fc1343a835` |
| `rate-e1-prod-cbp-500hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `bc6a4b5515d632f8ae31927ac926b03d7d2ab0c05a9e6f8850f22c8fba64a9f7` |
| `rate-e1-prod-current-100hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `6382d2e118666edbf6172f79d72688e0b53b7af28a58e2b1d64025a53efee2f0` |
| `rate-e1-prod-current-100hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `e97404c266999289e41bbd26ffe5c1c95adf5bb4e23f7b7be591c3851b523770` |
| `rate-e1-prod-current-100hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `58cbe08ce001f87ea651ca2c8e47c58e9109bcd27129ca4ed77b4c579343b42f` |
| `rate-e1-prod-current-100hz-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `aa113e6c7d37684b5f2b9ea221979abc2070b244d27b2ffd7d0e55f702dd8645` |
| `rate-e1-prod-current-100hz-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `95c1afba85c13ab99972ea5005cc3c68a864917d4956b963bac9190e7b8c021c` |
| `rate-e1-prod-current-100hz-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `e1195b131d49939236dffbc2bc9ac015d8a1440d976e34ae227d38db7ca2da38` |
| `rate-e1-prod-current-100hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `fc61dd5cb1a8a31c78dc4aa004ebb2092ca43ba8bf5d0eadf502beaa17e1e038` |
| `rate-e1-prod-current-100hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `49228db5fb4f4006faf3556bce0635a7365cf982d4c0f2ab28ecc66c87237868` |
| `rate-e1-prod-current-100hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `55f08a1687d930e4e1d5b5f48708f2f68df8dc6f1af858974194bd8692946f75` |
| `rate-e1-prod-current-100hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `79635c97001e948e3734e7c1df80a5ebedd2704a8cd218ddf115d9eea5cd44a4` |
| `rate-e1-prod-current-100hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `e885ae930e56131ac2a26b41f5b57919a6bd7d1f0f286bd86fd8c8a7df0a9888` |
| `rate-e1-prod-current-100hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `c77f73b5d544a2e3a6655b8b45a3964bf38bd72fbe86398c30a2bb4c21e7ad3b` |
| `rate-e1-prod-current-200hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `24d770282353f74c92bce6183f5ab2d0524a7c2ca56f5cef77a1add0952c1078` |
| `rate-e1-prod-current-200hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `9877959ec4b42cea2d3ae63d1185f764d3420943fec7ef498ad6b07b3ecdbcbe` |
| `rate-e1-prod-current-200hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `d9d359d173bdd511e7c264d51e455b81c9a6961476b4d41a4970d6b19eda4556` |
| `rate-e1-prod-current-200hz-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `8d0cda714a801137e1f1fa0f59f3162d9f3de2e64a383073ae9f58688f36e9ec` |
| `rate-e1-prod-current-200hz-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `846ad7f7a662a331b0ee04a8bc250eabe789b5e3e8686af66cfa3c988a89f1cb` |
| `rate-e1-prod-current-200hz-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `30f79c3269cccb194a386d9ebe201908f7807b42ba06c1c938f3943577e97cb8` |
| `rate-e1-prod-current-200hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `d33c4c86694f205d5e2279481e3d9afef6ef479a8f65fc3c27860e0bfcbe2323` |
| `rate-e1-prod-current-200hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `21d843b0d5fde7f6797dabb47559823c7b112eddc9975606b89ae8b91c019614` |
| `rate-e1-prod-current-200hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `3bdca88efec0bcca797b28f038de14fb3f99a402e09a1bf8f8fe0cb8362f2936` |
| `rate-e1-prod-current-200hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `d64774f3ae974df49cbeeeb1d6ae2111442c2491010ffd654043e9c78588bdab` |
| `rate-e1-prod-current-200hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `00f9eadc218e8f86e8f72ba3cdd5ffd51e31b36e3fe0ccc98aafd563524fbd74` |
| `rate-e1-prod-current-200hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `4d154c61cc608831e668a74029962adb4886f4631f939f6f9c44d49d60256438` |
| `rate-e1-prod-current-500hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `1a763d7b0d60a3af459de1976ea82f459221e3d4d9d7a9ca4283e53be9a51771` |
| `rate-e1-prod-current-500hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `b60e933a6b49b49395bd2e59873e8376e785da0a1291868309195d0312418c5f` |
| `rate-e1-prod-current-500hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `22cf27cbedacfbc9f402d5ca2a1fb31edf7b684a7a3c22a8f1546d3e47f11062` |
| `rate-e1-prod-current-500hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `2f0e4d5cc38b83345a71ff76a3679bee520117e27273935c5139aa8b06140d38` |
| `rate-e1-prod-current-500hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `f4e8d5e7f1ff53c34b79ffa1b21f1fc31a14956daef3dd271632474325e3c952` |
| `rate-e1-prod-current-500hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `1d032d2a49b8d66a9f5311b17db8f196a41e39483a6113ddd2859aa0bfcfbfa4` |
| `rate-e1-prod-current-500hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `1bef59e06504b42fd559aab5e2d306d8df35264c4c7f5948be2c3327afb3d059` |
| `rate-e1-prod-current-500hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `66772f88cd35f45482dfd8e2db46c51bd512e788fd96aad29309e6d215679b10` |
| `rate-e1-prod-current-500hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `e2e6141fa9b79a7fb5aa55397cdd4397df33a6a40897fa13a93e9c18b37098d5` |
| `rate-e1-prod-position-100hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `dd296593e30e34b2567bc2b01c8659c55e66c27689eb6cb107f314eec85792ba` |
| `rate-e1-prod-position-100hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `3933e486e5e3f80edb2c0ec322bb73fae15450051710a4d1f3a28df867457e53` |
| `rate-e1-prod-position-100hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `66cb7f1958a340509c85d5a1a79f85ee880763f28254636cbe9b483be36d0b9e` |
| `rate-e1-prod-position-100hz-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `d427329cb4c77e98162b3c20e71475fb89213af836955d49cd4117705a05aa45` |
| `rate-e1-prod-position-100hz-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `c3eec470e59c576ce5015fc7c0619381b2a06cb30c5975a4f7138f905f0019cf` |
| `rate-e1-prod-position-100hz-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `26de64e5f9e07446fbf7361e3d0fd76282978670ad387711fb71dbbae4098a0a` |
| `rate-e1-prod-position-100hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `fdb4965f76c6cffb40185dccbee4196ec4319c2109ab3fe1007ae9694b1213f6` |
| `rate-e1-prod-position-100hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `e927012c5e5638364bb87cebf1a6fc05ab2689fb79067289952dd6258a237b46` |
| `rate-e1-prod-position-100hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `06072a3b8b89fd8dfe31704eaaef394720799f5914ab20056f850e3839c1ba86` |
| `rate-e1-prod-position-100hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `f8a061f336a7985cc531f50d186bcf075015b05b86c708bffdc8283337c22916` |
| `rate-e1-prod-position-100hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `904a4f935382056487d40aa9d0e16bdc10b58b75925a2c0b6e318fe26a4b5894` |
| `rate-e1-prod-position-100hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `460eecd7ed210767819033038a03944cd919b591d374ed57758a8afaaf0610ef` |
| `rate-e1-prod-position-200hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `2299aa052a207f2fc95b39758ae786b05b0f30f146cf65b0cf74b64b1222aace` |
| `rate-e1-prod-position-200hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `b9fa28a25d89e31bcfd3d1a0c4ec5e74b9ddfa2ff8d63e74c36c4bda49155365` |
| `rate-e1-prod-position-200hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `beb5f4ec381269b1020b6d16414dc8c41f51eae1bda090c28349b66a91a16a68` |
| `rate-e1-prod-position-200hz-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `1ad93286197c4228b59df5eab093edbea10ef2e13366392c67948e0dc41776ea` |
| `rate-e1-prod-position-200hz-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `d1cd64e444e3a217768185831d525c8d81a69d6ed038b5aa192211cac710904d` |
| `rate-e1-prod-position-200hz-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `98b37b4be33f128e4df9c16863eb63edb24e4536dcf4f9b9ed54b0ba60058bed` |
| `rate-e1-prod-position-200hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `1eff7d18f3c74a9bb24030f6ac46e6dbbd67820331ae65343f0fe97a58f48fc1` |
| `rate-e1-prod-position-200hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `662ac0544b1233c2c29ebedf3e1d7c3d97cd6259d86e4594a2bc304736b00130` |
| `rate-e1-prod-position-200hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `1e0076bf2aeeb89ad3de999afe720168b1bb7e9dfccc03815ce3a91264ac2eff` |
| `rate-e1-prod-position-200hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `8c67bd38245ea3749291033af8567f675a12fcb5723214ac9464d759e11c48c3` |
| `rate-e1-prod-position-200hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `de670c3643a73fd22992fa45966da918c86597225c0429920b09d9bca4c1829b` |
| `rate-e1-prod-position-200hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `3391abad4a3981c72c630aa99ad20ec3654008cbad07916aeb934b9337cc85b9` |
| `rate-e1-prod-position-500hz-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `bd81e647b9da764504f4d6bca801a896c4d5df0a7a1bd1fd6c1da94c1a77d484` |
| `rate-e1-prod-position-500hz-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `83810a462c96f0646424e2310d7fb9f20b65ff3b884b679aef0e249206d89458` |
| `rate-e1-prod-position-500hz-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `2c87a31ce58fd2480c079629261862706cff8872cf9e869da640c8efd37bb1dc` |
| `rate-e1-prod-position-500hz-rt-idle-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `3325f988b1bf19afffadee40bec3b41501bf5dd3aa5101f6ad9005d301d8ba9a` |
| `rate-e1-prod-position-500hz-rt-idle-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `50e11ec3aad96cce9d3f1c5f81071d17c61a36b13f7bb1be3b08d07900ca3f32` |
| `rate-e1-prod-position-500hz-rt-idle-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `846ecd90d1d21a2cf3a31b314e596c16051682021a931f118da93270f68a4d94` |
| `rate-e1-prod-position-500hz-rt-loaded-20261103-1.csv` | Legacy-path torque-off production-traffic run, per-cycle log | `953315719466233ca9513a8d0d43db9860f4c394efe7b957c92383106d5d058f` |
| `rate-e1-prod-position-500hz-rt-loaded-20261103-1.csv.run.txt` | Legacy-path torque-off production-traffic run, session record | `964073a82075e2666c61413452b822aa1dcca523ad8fccb176bba7b6cdeda0b4` |
| `rate-e1-prod-position-500hz-rt-loaded-20261103-1.csv.summary.toml` | Legacy-path torque-off production-traffic run, probe summary | `e6b3b8c89fae42f00316054d8a0d27875befcfc1f4f4189b51177183a79c8779` |

## `e1_cal/` — control-rate E1 calibration, legacy read path (2026-11-03)

Attempt 1 (two runs, `tools/e1_cal.sh` at `dba323d`) compared goal
read-backs, which the torque-off servos overwrite; attempt 2 (11 runs, at
`a16ff42`) acknowledges each write with a Model Number read. Overruns are
expected in calibration runs and never judged.

| file | role | sha256 |
|---|---|---|
| `rate-e1-cal-cbp-100hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `9dda66ea8bf5de704b271b7cf6d0086c00f43b56a7ffd5735e91d1026802dc70` |
| `rate-e1-cal-cbp-100hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `93c87ebc44f6913d47538586f6dea4a5800c280ad0ad966f371010506eb915a9` |
| `rate-e1-cal-cbp-100hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `b79ab52df28ba879539d130aa8f855146d8f0eccf59ff294ca34afae964cf279` |
| `rate-e1-cal-cbp-200hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `a96a9c25f2234195be503513a23130f1e4398015ac98a743c0302cef6116e76a` |
| `rate-e1-cal-cbp-200hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `002fe1849d33ce073ae52ae6a18ab8c2a9383ad418f08c34955c75dbf0b83692` |
| `rate-e1-cal-cbp-200hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `0d6dc571726124db81c515af6886239287033bd1ab5895bb5a82bc09171ca663` |
| `rate-e1-cal-cbp-500hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `11a805ee77e0be9ee398d7ba9bb29eaaac4e0f8529c10d524a5d588830ab1bc0` |
| `rate-e1-cal-cbp-500hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `7b65cbfbbd771d1790594ecadbc5e3367181ca67f84703a45e889af41693625f` |
| `rate-e1-cal-cbp-500hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `049d784b00e930eddfdff976619ffd65abad1970de89b712e06181a616c79702` |
| `rate-e1-cal-current-1000hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `69264eff8bc93b632ab9a74572dc90de5890950704aec08e54699a9d3bb6e250` |
| `rate-e1-cal-current-1000hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `9c99200aaada285dd4525386ae0b15232aeca01c3adfa780049657132e1c0d7f` |
| `rate-e1-cal-current-1000hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `3faaf3b8a3ddd46b5116f42f2dde10f6feb32af27d44596c74cf243aa98c5ffd` |
| `rate-e1-cal-current-100hz-20261103-1.csv` | Calibration attempt 1 (goal read-back acknowledgement, summary version 1): per-write bounds only, no distribution; per-cycle log | `8fedf40d0414a1d1a95fa77a268e30afc734d844c70e6bc5ea8596057afe2bb2` |
| `rate-e1-cal-current-100hz-20261103-1.csv.run.txt` | Calibration attempt 1 (goal read-back acknowledgement, summary version 1): per-write bounds only, no distribution; session record | `2c039cce8fa559cae8511bdafe48b9324e7cc557562fe4c4b998ee15ad0f5003` |
| `rate-e1-cal-current-100hz-20261103-1.csv.summary.toml` | Calibration attempt 1 (goal read-back acknowledgement, summary version 1): per-write bounds only, no distribution; probe summary | `722ef615e3c7fd40381611f0528dba613dd46b806c959a78ddf974c379fbd934` |
| `rate-e1-cal-current-100hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `e1f712c5ebf5a4213111e50327e7bcec2d63f7d66dd9fa82dffa7532c34e6455` |
| `rate-e1-cal-current-100hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `c95905a0161de4777f4397d6cfad92aa84999dcd8fef874c25173bf64663edbd` |
| `rate-e1-cal-current-100hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `86965fa88a824db6b35e3a345a78467a67aa3c8b8661b50b2cb9da353b24a31b` |
| `rate-e1-cal-current-200hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `8d720a06cbfbf73677095dd28bca56a1f639e17d684a51005fd77fd8f3e1d989` |
| `rate-e1-cal-current-200hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `e4e45e061d13c671d368e9d4efe3f000e025e0a188eb78ec3a46eb7026f5e39b` |
| `rate-e1-cal-current-200hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `cd096414b91d32b88daa8b4f55a690b8e0a878a8962e2fedc270364c90f3cda6` |
| `rate-e1-cal-current-500hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `3f072e3e678137888d56139caf0239d958c3c9d0889e6e2b76590bbf1804fd5f` |
| `rate-e1-cal-current-500hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `a46b4434e6b61c3acd267659529337f876f0ac0dfdda671e8a55c518ce53456d` |
| `rate-e1-cal-current-500hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `ed0a25c712d7044023b01286b3c4ed50dd4a54cd5aac685d10591018133a9167` |
| `rate-e1-cal-position-1000hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `a8a882c1551424b861ef366d2e94b1b146268a62e8699ddcc5a881ef8a0ed835` |
| `rate-e1-cal-position-1000hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `4b91918f8afd8f71a68bdf45d6567a0710b23cccb0ed07eb41940216e45835bf` |
| `rate-e1-cal-position-1000hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `8a9a9a905eab15130931b26e2512dad7dca7f3e099726a905e59a6ba8dde9ffc` |
| `rate-e1-cal-position-100hz-20261103-1.csv` | Calibration attempt 1 (goal read-back acknowledgement, summary version 1): per-write bounds only, no distribution; per-cycle log | `0ee10e7d646823b92578128ef12a58e2ba42876b1e100c13d5e9a9cf223db475` |
| `rate-e1-cal-position-100hz-20261103-1.csv.run.txt` | Calibration attempt 1 (goal read-back acknowledgement, summary version 1): per-write bounds only, no distribution; session record | `84e1ce76df00b9820d20946f6a39df481ed0315534256234933763450f6c362f` |
| `rate-e1-cal-position-100hz-20261103-1.csv.summary.toml` | Calibration attempt 1 (goal read-back acknowledgement, summary version 1): per-write bounds only, no distribution; probe summary | `33b3380a231b50829695f4e64cb6a475bf216f24e96544b56c44d5d78a24e64b` |
| `rate-e1-cal-position-100hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `b49f107c2d8be4c58bed32c3b445ce0e36731ae68d3d453bbbf1258aee96f6ef` |
| `rate-e1-cal-position-100hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `c2e6cccbf5e31dd7c70a3018e6005a3400e74437a55b47ec948f9077a6163bdd` |
| `rate-e1-cal-position-100hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `bd11b92b48fcfaec4827369fc102e509c5725dc56fbb746cf5a84a8f56af1a32` |
| `rate-e1-cal-position-200hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `dd6d939e2c51a3c61fe163f87f16510209135ce942a92d2dc0651b451bf61711` |
| `rate-e1-cal-position-200hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `d5deb52d8f1bfa1ee9030e3c02506a1eab6e2ec7ae29b91efb2da9cadaa336b3` |
| `rate-e1-cal-position-200hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `498355a2ecc9a101912c616c275ff15f210124258c17ede9b6f77e185236a751` |
| `rate-e1-cal-position-500hz-rt-idle-20261103-2.csv` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `330a576e685c6021a53f0f21571abc24a4570d1921984932ebf35ab3a732aaac` |
| `rate-e1-cal-position-500hz-rt-idle-20261103-2.csv.run.txt` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `fffdd55b4cb295cb8b8a78565e5fce5b806b8aa27dd65f4539e97811715a73f2` |
| `rate-e1-cal-position-500hz-rt-idle-20261103-2.csv.summary.toml` | Calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `f65cb20ff095efadcac5216d47c9f7dba56a6391c24aff12e29858295b41fd06` |

## `r4_bounded/` — control-rate torque-off runs on the bounded read path (2026-11-04)

48 production-traffic runs (`tools/e1_prod.sh --read-path bounded --no-climb`)
and 11 calibration runs (`tools/e1_cal.sh --read-path bounded --rt-priority 80
--rt-cpu 3`) at `5f73a08`, with the 35 ms budget and an equal guard; three files
per run. The evidence for closing Phase R; cited in
[history.md](history.md#control-rate-phase-r-on-hardware-2026-11-04).

| file | role | sha256 |
|---|---|---|
| `rate-e1-cal-cbp-100hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `f00c49c27996725dc61b5c613c94a0c219e4370b5616326ef71fe857d9587e2e` |
| `rate-e1-cal-cbp-100hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `1314778741099e18944603c0d9239ba505de4a6da412e5d7a427230950b6f081` |
| `rate-e1-cal-cbp-100hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `fac3fd3936df65c8fa146031bdada979127cb00a0204c07122b9c6bfa30874f8` |
| `rate-e1-cal-cbp-200hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `c4106fb3893f034cbf4c25d61cba6a9d14d3b0b257da4f7d7cc55526bc8343e6` |
| `rate-e1-cal-cbp-200hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `ad15e28e81cf377fb4d2b22b7b50b4fff5fdf40226f5899965e0851940923fe7` |
| `rate-e1-cal-cbp-200hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `2f2c8b17da6d80218eaa2334ec48dd59a44c48645d3cee28b578c10f967160ef` |
| `rate-e1-cal-cbp-500hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `5b04fc3c1133b2181c091f3d86b277a66d24f6a0f5880f72f941792eb1ca86c8` |
| `rate-e1-cal-cbp-500hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `2e25d8d472d95e025203fbccb1eb995a38b8972acdd374632d96098311e2f110` |
| `rate-e1-cal-cbp-500hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `b2de2ac83b9ccc37f2a38318f1dfbedcd942797996c5912393a3b2e7936115f5` |
| `rate-e1-cal-current-100hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `eab11a69cfe50be13e5ad5bf28e1b07f6b138a2e6f6f149128f853259880e8b9` |
| `rate-e1-cal-current-100hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `747cb158bdaf09c2d5e939429642d0eabd509f29dbff215b5f766f4e83e8e046` |
| `rate-e1-cal-current-100hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `f9eff5d984203ec1ff8f405926eecd43f39a74a857cea4f5c94deb8a83c8517c` |
| `rate-e1-cal-current-200hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `f09039d6dc2c047e0084814980fea91e7fceab4155fea6f65bcf638c8e2e6edb` |
| `rate-e1-cal-current-200hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `09d47c825935293b0b03c43201a9d2add3ea7a99946a138d77b74776e58678ef` |
| `rate-e1-cal-current-200hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `e9170b4ee3f81f6ce99e94e3940f6c766bbdd8aef4b0067ee722e9aaba049566` |
| `rate-e1-cal-current-500hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `ac2f9a23330bbacfbe9939d1e62e8dd54d0d22d9f4fee59aff95889f51c0df3c` |
| `rate-e1-cal-current-500hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `944b0915251bef08c98a5e3cbde27a7cb12e7b14355f99267962d2e1bf4b486b` |
| `rate-e1-cal-current-500hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `3bcf23d58fb64c764cdbb58240614a01647d66ba37f3486dae5ec41db4261bcf` |
| `rate-e1-cal-current-1000hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `c2c743354b9681ddbe6dfa947fdad3b357b336e346eea058d6d96c41f81bbd80` |
| `rate-e1-cal-current-1000hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `67435610bea904ea73d40e7e6b6b78c893d96fb701e350781d8ed0c4f812ae65` |
| `rate-e1-cal-current-1000hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `0abfec858cfa5b81609e9e8316d390b1793be30796d2862003fcdd5b0959dfee` |
| `rate-e1-cal-position-100hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `04a3b37b41bdb5e6aa8edc70e5c5aee76bab7980e50c13a6f731c2b4ef1193ed` |
| `rate-e1-cal-position-100hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `35ff46a7bc49355166e3543c8294a4fae8381d5e6fc6003803b53eff6bcf027b` |
| `rate-e1-cal-position-100hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `7291193d4a20f5c817ce703343998319ba1de2aaf0471234ea7f26943df01093` |
| `rate-e1-cal-position-200hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `2b72013e9c364b5816de468925ef1eb8294d04dd89efe5369ad12a05f0ba4732` |
| `rate-e1-cal-position-200hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `ffdc08c24d429407e0a9bfb4220393aee0fb759d8955f4697c83e5ecee19c3e8` |
| `rate-e1-cal-position-200hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `a27665a3180de20ddcb906152dedebf7945359846ab1ab49242c05c5d67e6c25` |
| `rate-e1-cal-position-500hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `60fea0957cbd8d4efc0cbca560e4cbd76ec3655ea7e8186451809fb83de5d0d8` |
| `rate-e1-cal-position-500hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `3c45ab5aa50f7ed26b3b624591fe67621f25109c8d1db8b92c8899c3004af34b` |
| `rate-e1-cal-position-500hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `65aa20b353bd32e6b1277212dc1298da776e9a5c26f5d7baf3129b6e519614f0` |
| `rate-e1-cal-position-1000hz-rt-idle-bounded-20261104-1.csv` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), per-cycle log | `3b503048558c61312793b62c791a6c2937ceb11c9446e5c7f1091a4a016d2605` |
| `rate-e1-cal-position-1000hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), session record | `da7a1918e74441008b236646148aea53d5e46b32ae51b00324aad49a96c25b2d` |
| `rate-e1-cal-position-1000hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path calibration run (Model Number acknowledgement, SCHED_FIFO 80 on CPU 3), probe summary | `7a8e24fddbf9de1719d4eded7ff51c53445143a06c7ead1dd77637200e7f41b7` |
| `rate-e1-prod-cbp-100hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `671cef6e6dfe7715f1ab1bd1f9e4413196123802e5af585198ada70eb9a74f5f` |
| `rate-e1-prod-cbp-100hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `60210517fd48dbab95bbfdb38880e05e8eeba28c8f5bfb55b1a8a3acea455695` |
| `rate-e1-prod-cbp-100hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `6e1b9f1b26674da1af84481a0275abd118d33a5088f028dcafe5e0decf8de4e6` |
| `rate-e1-prod-cbp-100hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `60d8ec4e70cec6a15a7ff1f9b5fe06cc10bcb7863f0035167d40fced51dd7e87` |
| `rate-e1-prod-cbp-100hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `32bde24df342d6aaef7c22d4e29650645380d55e593837e4d63d438db274cd4a` |
| `rate-e1-prod-cbp-100hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `f33356ae18d5969c0a10ff0e3a71aa2b5ff287908f1e689b6547d0e0a7efb290` |
| `rate-e1-prod-cbp-100hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `e547dcaa3abed7e4ed4fafa3b74acef0e8216c9333da5b859341ed659d587c21` |
| `rate-e1-prod-cbp-100hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `81d5702859865851c57dbd0e54ab0331e5744823304f83877e57143098844e6c` |
| `rate-e1-prod-cbp-100hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `691d7868bd1f3cbdf73ded2c36c4ea3afbe00116a343476cebf99df35be001d2` |
| `rate-e1-prod-cbp-100hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `c5114ce684c2ac72358401652f5ec19dd0ed78da8da4b7b885d919f2e155cb17` |
| `rate-e1-prod-cbp-100hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `bb0cb7d751ec77d9d5f62338bedccea37df6b72094e5a3cdbb8072b04c174d15` |
| `rate-e1-prod-cbp-100hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `8839488b6fbefb58273364a6220b6e4460b9ea5878a7f7f4328e7cca3b0c208e` |
| `rate-e1-prod-cbp-200hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `f69a16c1132d9acce4a2d5b94381f50e2b61531b0c6295e41f0d1e424f093e95` |
| `rate-e1-prod-cbp-200hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `56af3ea0dad62d85bfb7306ae5e1af7628d5ed00bd4d1de6a03c39c0ad973c02` |
| `rate-e1-prod-cbp-200hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `9a5c7544c004a3a57ae97710a47409791de70626c9f0002ff89bf092843285b2` |
| `rate-e1-prod-cbp-200hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `531608a624d92f9fd5dcb50805d05609108cb512e9d76981446495466199de86` |
| `rate-e1-prod-cbp-200hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `498e4acaf0c5de657c79ed88105092adec65e835db83b492b0d8a18fcef80bfe` |
| `rate-e1-prod-cbp-200hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `5ec42093f6162175c56a8d002d91ce25da22d6b5aad9ac78359c93d8e2dde7d1` |
| `rate-e1-prod-cbp-200hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `3e7d4639d3ca28880680fae9f4b8afac95457fb6dd6bdf7a20309d99d04534a4` |
| `rate-e1-prod-cbp-200hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `45b8b87ac97cc9430024d7a366fa38403630d89897255a07ab8789994476223f` |
| `rate-e1-prod-cbp-200hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `8a807a3e48bbcb1355329ebfad7129eb908136cde913e1a0d1b85e1b7955e631` |
| `rate-e1-prod-cbp-200hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `4f5d504246fd67cad39f14d97b055bf544c0acc616f8f66aeaf05b9627fa49b8` |
| `rate-e1-prod-cbp-200hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `c58ae526ec93e9a095848b6df9b1aa37c81ae13b7cd3ce4e199252e3b2fb0abd` |
| `rate-e1-prod-cbp-200hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `1a26188ba787f1f8847015e33f385220b00c073ddd96cda1bed959091aee8e7e` |
| `rate-e1-prod-cbp-500hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `890ae600e92a566355c5bd51cb1446d6a0d6e39156d582ee0cbbb230e5738c65` |
| `rate-e1-prod-cbp-500hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `d7a4603c350bada11bfa3d216c91be8d4e95bf83f5ffe09bffbb83b512467d64` |
| `rate-e1-prod-cbp-500hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `6ed1db168b737bf4d6646c49bc80d739d58a690b98eea44c6a440f279f8ff2f2` |
| `rate-e1-prod-cbp-500hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `8a7421898b706ffcd24a392609813176a687e6bd2b4339f21c2f4affead44d39` |
| `rate-e1-prod-cbp-500hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `c4edfcb8b3a105e38b8e757e62623807c88607e55529bab940603eae5cbf2c59` |
| `rate-e1-prod-cbp-500hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `b3035150681582e90ed28098802695c1ac6aebe92beb10eba422d98fceaef2b1` |
| `rate-e1-prod-cbp-500hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `8ef604ae70e0215e9b81f0149ee8d034acf2f480bea16e6746fa8071544465eb` |
| `rate-e1-prod-cbp-500hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `da63ff396eafef3155a0d39afe33548b4fcb7be7f9baebf5a461f0081867bd3a` |
| `rate-e1-prod-cbp-500hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `bfb401df90af449c935379ca45dc06f754423a35b63d9dac8aed708302d5dd5b` |
| `rate-e1-prod-cbp-500hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `f589bddb6b68320ef6a64f0a21a12e48b1990a1c33ffbdb23ca969a687dfd201` |
| `rate-e1-prod-cbp-500hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `c213f4c9d373caa09c78ed6cc267d2eceb226fd05b44d57aee473001ed968600` |
| `rate-e1-prod-cbp-500hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `b8045fbb0be56f3813b89231a412ffe6d490dcd501435679c9ef14f2f9d6532d` |
| `rate-e1-prod-cbp-1000hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `06db56fbcd1ebc298fbb08b1244d761e313400fdf94fcec52892177014dc2a05` |
| `rate-e1-prod-cbp-1000hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `e1c222d5abf18943d7ad59e95b3310be952853e97271971b5b26382a5451d0ec` |
| `rate-e1-prod-cbp-1000hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `24856b92ca10eb083ff3aef062e6fa0096ad6d3889452257b8aae2fb989c553c` |
| `rate-e1-prod-cbp-1000hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `fb2c994316e88e8b9a7f826824722e0c7f611bf6c6cb51a86ae4d4ca721d3147` |
| `rate-e1-prod-cbp-1000hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `492fdc8844c3302ac6afea635cd6f42b43af719f501279414cd36496fceaa274` |
| `rate-e1-prod-cbp-1000hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `e352960aa1fb9cf250b5a5b44998ae2063da0ca9091ab704e6f5d4327ac48a00` |
| `rate-e1-prod-cbp-1000hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `4778b9f9f0f60970c424b37e0d81622cb1836ea63f61c7196da9760a6390b563` |
| `rate-e1-prod-cbp-1000hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `530541d1ec8f0fc4dbe5b1f14c03af0664ed2995156b475ea1d0ff3417212dfc` |
| `rate-e1-prod-cbp-1000hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `2481e24cc951249bfc29342cd655dff8856e7d94512774b076e80931f11045f3` |
| `rate-e1-prod-cbp-1000hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `75edf489e1a1db67253494efa80e7aa543d9d9fe5bb749892285cbf193c82f54` |
| `rate-e1-prod-cbp-1000hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `5474950b71824c9f460381af5995180c6e75fe592525012f1d1d2e1ae38871c0` |
| `rate-e1-prod-cbp-1000hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `3fff719d5dfba7b23544d51c5ae9623fb068d308071dba9ea616105964455388` |
| `rate-e1-prod-current-100hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `64e9d6078c76cd8fb60b3bb469e86f31fba2b6bb6a2e15b5fee6259d92271d91` |
| `rate-e1-prod-current-100hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `e836e29fb6368a39d1501e27c5de50f82d9f4d5d5ca8b5bcfaa1a484421a14aa` |
| `rate-e1-prod-current-100hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `a73444da9c4b90cf23af2cb27336c582a6ab980ff19227b029d85862fe3389fc` |
| `rate-e1-prod-current-100hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `d1f28882bcccd23725940102fb2c91ec36c11a4868e6e296fa7a6c44b3ab1f70` |
| `rate-e1-prod-current-100hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `92e3931028520662ede7e0e8d9270a5fcbb1570c0492e6b36151c6c7869e6cab` |
| `rate-e1-prod-current-100hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `0a9ae191e1520dd1203fa270ef272e4be03ca921f7c4de3b1d6f2f3b9bf8a876` |
| `rate-e1-prod-current-100hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `86272b87b7cb507dbb14dc63b98788e19524b6b0efdd838b938852c27f11d9db` |
| `rate-e1-prod-current-100hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `8cb56708f04a08aaaba3fe536cf63337fd4200baa90d13aceffa87f5a9247ff6` |
| `rate-e1-prod-current-100hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `130555da48a3720e4da91cf6a97acc2710a3aa9a95b374ce09c7e1d29fe5ca6f` |
| `rate-e1-prod-current-100hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `6877b1eda81471a90d59fe4d7c4fb6fe3aaa704663e4c01dfb16f25382562869` |
| `rate-e1-prod-current-100hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `cbddeea35db262148168d8ebf74393c5494463fdb953d183e5006cbb12ac03bb` |
| `rate-e1-prod-current-100hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `5af0b1440a35f5b12019f82507dab650fe0731a8a9491a2da60dc79e295f9ed0` |
| `rate-e1-prod-current-200hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `81cbbda002d35cad2c0fa5a2b041d087da8aab09c8e6bb8108b43b01965c1590` |
| `rate-e1-prod-current-200hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `2c2ab0e872f1d3e4e94d956d1b4ac26640bd5244d0e1d6ecea3b9dbae5975826` |
| `rate-e1-prod-current-200hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `111b62183f88cc06ec12e65ad9f6031f0b9148304c29ccc51c4af46828a5295f` |
| `rate-e1-prod-current-200hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `6f9404640d964833154daed3462487709c3fd09deed2f5ca6294aa112be67ba0` |
| `rate-e1-prod-current-200hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `524e8376d7cf5ec9fb5c2f10e9e166f2237f01a68cf29c35cfea887eadf830a8` |
| `rate-e1-prod-current-200hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `c9f0bef53ca1e351700c13cbba6b9763811eb8e134f6be9931c83d536d1f2ce5` |
| `rate-e1-prod-current-200hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `8827be0720471878b75db66523a7d989ea63014d11e432825e3b3a7f260dff13` |
| `rate-e1-prod-current-200hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `7b3166b9a89a01f6d7d7dd3f696942365342701c64e855652587847667ee59cf` |
| `rate-e1-prod-current-200hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `10423c7e412863211d69e887475aff106138e085de86482d2f5f97f5966fc1e5` |
| `rate-e1-prod-current-200hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `1fb7a6893d084534abf70041007ed8b6c2222e8811ef45cad0cca2313f9c1cc9` |
| `rate-e1-prod-current-200hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `bd316157aa17cb91aa3c4991c6d37409d37d912daa57adcd3236dd180c8f6f47` |
| `rate-e1-prod-current-200hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `af189aa083bfdccf3e9647bd5807e4a260cea3915dbe394b8f095646ed129782` |
| `rate-e1-prod-current-500hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `cea8afc82d086dc2ceb0b8870cea30dfa95b53df91f456bf89648798e06faf8d` |
| `rate-e1-prod-current-500hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `bae01aece0622c1725a45a0baea1e2c78c09120a51c1d3e49b6976c735667c55` |
| `rate-e1-prod-current-500hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `a9c87a959515b0488d042b88cd5dc3cdf630c344a56b4411b88665a3aa0a04dd` |
| `rate-e1-prod-current-500hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `51e13a5a0b165968b955e24e9ac39a2cb71eb647ece4bd6dedded025200d5661` |
| `rate-e1-prod-current-500hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `5fd4126c742054b8c73c0e7417f9e4ac321fa44cd51141e4f95a9e179eb70bec` |
| `rate-e1-prod-current-500hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `27477a21b4784f6f7d6f351b441fcbd575ed6767021f9b59505d8db0bee2b9c9` |
| `rate-e1-prod-current-500hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `f1515005a46b653398017fbffcc754650be6825f3624504baae8df58b7ca1cbe` |
| `rate-e1-prod-current-500hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `1095436bde158536c1b44b5d3f438957b60a92d9d950bb44532a146eea81fed7` |
| `rate-e1-prod-current-500hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `b18e393bc7df09f90f630d40fc4249c2f6cde39655ea5a425cfc02579c8fa4ca` |
| `rate-e1-prod-current-500hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `76d73169cfff2e6d450bebcb255f9d7c0ede82cc86d375a7a65fce4308e9177d` |
| `rate-e1-prod-current-500hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `87e843af886992dd2b77ef63012ca0842bdc3da91b53059638809ec2f16eda00` |
| `rate-e1-prod-current-500hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `1d9887b8b46b24d438c457a84e1efb41fd186fde4a4cbd5f61e635eb1eeb92f0` |
| `rate-e1-prod-current-1000hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `83ed9529692a7b35f4fd7e636ec2b986b697a23946f99ec14d878eb9cfe81efe` |
| `rate-e1-prod-current-1000hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `cd544a1a05e1c5f1cd9d1fca5934fd59ca84b2ff68cc2ca0744775e402a3c4e2` |
| `rate-e1-prod-current-1000hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `f48365a84ea5fc08063b8331fda0b5a232e9a44495819daf4e146004ca49957b` |
| `rate-e1-prod-current-1000hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `cc68ece37e041e994cfdadd50c77757c2d7e117f57d3645478e07e247381f234` |
| `rate-e1-prod-current-1000hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `de67e778f68638ede0711b8d58f835cd57eb3794d5006985650f739af0390ce8` |
| `rate-e1-prod-current-1000hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `678d81048abebb874fbdcd0823afd179a7afc1e3fab074c5df82be874fd281d7` |
| `rate-e1-prod-current-1000hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `05584337d7d0bd129ac3d873688975a98c440deec43f8e36424edc17bf2b4462` |
| `rate-e1-prod-current-1000hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `274e35f96ec0080650d1e930fc94ed1033a9f81532e84bbbb35e8db1d0cbe314` |
| `rate-e1-prod-current-1000hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `b68ece5705801d38e4fca59eb561077825757e47b1c762910ec37a69e8b7ad34` |
| `rate-e1-prod-current-1000hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `bac18970212ed11ed7797d6eccb45c2de77985569b5498d4bbba94100d5a6074` |
| `rate-e1-prod-current-1000hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `f75bbe2c4603e40d7e96900be88f289448381ca1475727a7c7a37e43becdd7b7` |
| `rate-e1-prod-current-1000hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `a1b359fabf030fca9f0d4b6eb0df9687ec41fd0c959f705e4eb0037adcbfff68` |
| `rate-e1-prod-position-100hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `b0c7758ae0f003fd08e5216cf77dd879011f20a6193500216e2749d0baa2f7e8` |
| `rate-e1-prod-position-100hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `0020e92aa20f76fdf3124ecdc992d37c3ca673dfdcb5c83c4b430c6134b1bbd5` |
| `rate-e1-prod-position-100hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `2ff3bce47e9bb78bc61e703c9ee97b53c1c91c755e37bac9143a94a109439320` |
| `rate-e1-prod-position-100hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `62ffb41f1ba1f00f8eafcbd238eb5105c81039d5aad7829b79a34ee4168f4558` |
| `rate-e1-prod-position-100hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `51084997e735301fd14fe074581460b9250abe41877ec562f3b2de8172a59365` |
| `rate-e1-prod-position-100hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `6c4fcce449407485c39c72d2447589b0a3f238c7b837e545d4435f62181fba9b` |
| `rate-e1-prod-position-100hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `49fafc9ebf203429b3cde3a17fded62947818e5146509d8eed14e62460056c59` |
| `rate-e1-prod-position-100hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `90ed86f1626cdc7eebf63ea627cb5034ed23a75f9e7f9ef7dce6dce31151fda9` |
| `rate-e1-prod-position-100hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `8d3911b9f41bf87f17d36083168dc07b23bd27c1826437faa0f91504d934ae34` |
| `rate-e1-prod-position-100hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `2b60e788f056322859568208e70e0ea0f25f314b7577720bb8945252b96f7b89` |
| `rate-e1-prod-position-100hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `b66d244a4d95acfb5bf0a79c232d713c31cc1268d03935bfef56a5cafa4bee7b` |
| `rate-e1-prod-position-100hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `f88554391222e345ccee7997ba0fdb88ffd8d55cf6b3602caea14f37849dfebd` |
| `rate-e1-prod-position-200hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `e4d629deac90580ea4a5bb1582f10bac227e3205e544a03998af25dbc9194887` |
| `rate-e1-prod-position-200hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `97145ad089a3faf48424d565d5398cbb1b505e2ee8a7bdfe50a65e258a1fcd7d` |
| `rate-e1-prod-position-200hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `08a413233d49b72516bc9f9db350333ff4e121febe9862bf4124deffa6e77865` |
| `rate-e1-prod-position-200hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `372d571f7253a4c92d5474955314404660e4fd259be8b1ee3ee0dc79b29da32d` |
| `rate-e1-prod-position-200hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `41f2a0ba27c6f3863869100893199a48eb0a4cc890bb446e1ced1154f576b982` |
| `rate-e1-prod-position-200hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `295c2400670ac6a725921ab908a8d856b84605003e40144b08d050cad69cf4df` |
| `rate-e1-prod-position-200hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `895a1c410c768378054a1c18ccf91643bfc38407a60ffabba69ac2a285c63feb` |
| `rate-e1-prod-position-200hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `4526540d5129895416008c04075bb6d6e6db3052266d785c43e7a00fb4067a8c` |
| `rate-e1-prod-position-200hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `b87891e3f6ba6d2295bea03a9d6ce9c843a8436a37e3a5b87acea109766b954b` |
| `rate-e1-prod-position-200hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `b03840efbbb945dfc1e960fa97a02f9424f1848130719a2c045e177625aed1ca` |
| `rate-e1-prod-position-200hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `1cbc4d7b5edc9bccf56bc5797a9cc18af85e2a9535a23389754d4540d21561a7` |
| `rate-e1-prod-position-200hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `53ca354e3f6ce49de870cdd11a1f21c8296a4adad7cccb247202e7a706052c37` |
| `rate-e1-prod-position-500hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `0e24b150ae7d105c50f57a4f959e626030997eaebe768cbb7210467c894b8ac1` |
| `rate-e1-prod-position-500hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `c5720ff1adb5a60405bce3c9b63f959967536631b8493692e450023313a3ec13` |
| `rate-e1-prod-position-500hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `68d655cff56da1ed42378bd8e720aed6580872995b60dd797d349a4bfe7ef89f` |
| `rate-e1-prod-position-500hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `10fbffcf1b6c9939c82f7668d5f157e4e4795536a1122ef5605ab9ebf97b388d` |
| `rate-e1-prod-position-500hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `d76e1d8ae5be5e31ce0b5db5fce7cba7459a644a8acda5f167cb32e3a5f188b7` |
| `rate-e1-prod-position-500hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `cd36ce05b6af64f6af5fc4678d6c68afe0aabf4037e708d7eb8e055174a977c6` |
| `rate-e1-prod-position-500hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `450b92bf517900a850d806cbd4acd323a91e93141fde7391f2f67c109d3e61af` |
| `rate-e1-prod-position-500hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `91d70c71c43b09fc31fc94929045e16ad2e9dc63baf831f8a9c69c284f7763d5` |
| `rate-e1-prod-position-500hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `c20c1b5895f9b782c786b2924979d57d4b3ce7fb87af648dd598572395f9885a` |
| `rate-e1-prod-position-500hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `fd9bde97d58c1005ca1b5edf4d029f96925166c7efe39b81012db2a9f1c9c2c3` |
| `rate-e1-prod-position-500hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `bc06f87391c010c63c6c08d4c32ecd725e904db69e06cfe6837affc5f113c78d` |
| `rate-e1-prod-position-500hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `25e7853a686eee350e5f5d6ace603c7142eb0f2cba2d3a711290790f2c05f452` |
| `rate-e1-prod-position-1000hz-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `aefb45c5c5d245e345200e511198d026bdd98cf561c915e96dec078ab85d8730` |
| `rate-e1-prod-position-1000hz-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `cd36e0d4a38629af15928ecd1da7feac05091833f660ff6315260f3c44b7f58d` |
| `rate-e1-prod-position-1000hz-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `60581e9f00b3436f1e2c03c49f06344b9ef34282c1742908107f994b231c9896` |
| `rate-e1-prod-position-1000hz-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `223c2e848ed5e359e9f73235b460f0a2f2a384f791db17e5e1f746112edc9c04` |
| `rate-e1-prod-position-1000hz-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `8b259e87c1561e3a434abf1fc40829adefdc9726eadb8cdb5a3c4d326e97f694` |
| `rate-e1-prod-position-1000hz-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `f2c79d6410defa822628a8610da2775decadce4acc1ad734ca592effa955c89d` |
| `rate-e1-prod-position-1000hz-rt-idle-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `f2935038815f4d4a863abd23291aba0202517078f3e9c0969db60ced7d97cba5` |
| `rate-e1-prod-position-1000hz-rt-idle-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `a76b79fc761fbacc554336aa8d9be74e41ec014e95bc3f0e2731de833ac303df` |
| `rate-e1-prod-position-1000hz-rt-idle-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `3c9a8384ca92a7076e38888ea73349f5024bfd781a8e1f8572cf2fb11adbe78c` |
| `rate-e1-prod-position-1000hz-rt-loaded-bounded-20261104-1.csv` | Bounded-path torque-off production-traffic run, per-cycle log | `b36c7115106d68a4de7b2b34c20950685a2e3ce0cef290c7f06e17684db857b1` |
| `rate-e1-prod-position-1000hz-rt-loaded-bounded-20261104-1.csv.run.txt` | Bounded-path torque-off production-traffic run, session record | `266c3bc51e0ebd71575eaef0888d6c496b00dfc9cda621fab15f8e006d5f4672` |
| `rate-e1-prod-position-1000hz-rt-loaded-bounded-20261104-1.csv.summary.toml` | Bounded-path torque-off production-traffic run, probe summary | `484e92366dbc31b47e9686521c426d596d9c3bdef714c080f4799d7e97e75e30` |
