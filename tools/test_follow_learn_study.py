#!/usr/bin/env python3
"""Regressions for follow_learn_study.py, run under ctest.

The pure parts on their own: the ZVS frame cut, the TOML writer, plan
parsing (inheritance, criteria sets, sweeps, refusals), the derivation of a
case's configurations, every check on synthetic curves, the observations,
the table and the summary. Then plans end to end through tiny stand-ins for
x7_plan_ptp, x7_follow_sim and x7_follow_learn (same arguments and files, a
first-order lag plant, an ILC and a two-regressor feedback-error update):
a converging case passes its criteria, a refused update stops its case
with the refusal recorded verbatim and nothing retried, a perturbed trial
and a transfer run as planned, the parallel run gives the case results of
the sequential one, and only the three allowed programs are ever run.
Last, the shipped plans (config/learn_study/) parse, and the acceptance
plan's criteria and settings are pinned as the study's brief fixed them.
"""

import contextlib
import io
import json
import math
import os
import pathlib
import sys
import tempfile
import tomllib

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
sys.path.insert(0, os.path.abspath("tools"))

import follow_learn_study as fls  # noqa: E402
import follow_learnability as fl  # noqa: E402

SCRATCH = tempfile.TemporaryDirectory(prefix="follow_learn_study_test-")
ROOT = pathlib.Path(SCRATCH.name)


def raises(fn, *args, contains=(), error=fls.StudyError, **kwargs):
    try:
        fn(*args, **kwargs)
    except error as caught:
        for text in contains:
            assert text in str(caught), (text, str(caught))
        return str(caught)
    raise AssertionError(f"{fn.__name__} accepted invalid input")


def run_main(*argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            status = fls.main([str(arg) for arg in argv])
        except SystemExit as stop:
            status = stop.code
    return status, out.getvalue(), err.getvalue()


# ------------------------------------------------------- three digits

for value, text in ((22.4351, "22.4"), (5.8149, "5.81"), (0.12345, "0.123"),
                    (1.0, "1.00"), (9.996, "10.0"), (100.4, "100"),
                    (999.6, "1000"), (12345.0, "1.23e+04"),
                    (0.00012345, "1.23e-04"), (0.0, "0.00")):
    assert fls.sig3(value) == text, (value, fls.sig3(value), text)
assert fls.mrad(None) == "-" and fls.mrad(0.0224351) == "22.4"

# ------------------------------------------------------------ ZVS cut

FRAME = "0.01 9 ({} 0.5 -0.25 1 0 0 0 0 0 )\n"
ZVS = "".join(FRAME.format(0.001 * i) for i in range(100))
frames = fls.zvs_frames(ZVS)
assert len(frames) == 100 and frames[3][0] == 0.01
assert frames[3][1][:3] == [0.003, 0.5, -0.25] and len(frames[3][1]) == 9
# frame i starts at i * 0.01 s: those before 0.6 s are frames 0..59; a start
# within 1e-9 s of the cut counts as at it (0.6 is not exactly 60 * 0.01)
assert fls.frames_before(frames, 0.6) == 60
assert fls.frames_before(frames, 0.605) == 61
assert fls.frames_before(frames, 0.001) == 1
cut, removed = fls.cut_leading_frames(ZVS, 60)
assert cut == "".join(FRAME.format(0.001 * i) for i in range(60, 100))
assert abs(removed - 0.6) < 1e-12
assert fls.cut_leading_frames(ZVS, 0) == (ZVS, 0.0)
uneven = "0.5 2 (0 1 )\n0.25 2 (1 2 )\n0.25 2 (2 3 )\n0.1 2 (3 4 )\n"
assert fls.frames_before(fls.zvs_frames(uneven), 0.6) == 2  # 0, 0.5 s
assert fls.cut_leading_frames(uneven, 2) == ("0.25 2 (2 3 )\n"
                                             "0.1 2 (3 4 )\n", 0.75)
raises(fls.cut_leading_frames, ZVS, 99, contains=("fewer than 2",))
raises(fls.cut_leading_frames, ZVS, -1, contains=("integer >= 0",))
raises(fls.cut_leading_frames, ZVS, True, contains=("integer >= 0",))
for bad, text in (("0.01 9 (1 2 )\n", "2 values, dimension 9"),
                  ("0.01 2 (1 2\n", "not a ZVS frame line"),
                  ("0.01 2 (1 2 ) 3\n", "not a ZVS frame line"),
                  ("0 2 (1 2 )\n", "dt must be finite and > 0"),
                  ("nan 2 (1 2 )\n", "dt must be finite and > 0"),
                  ("0.01 2 (1 inf )\n", "not finite"),
                  ("0.01 2 (1 x )\n", "not a number"),
                  ("0.01 2 (1 2 )", "no final newline"),
                  ("", "no frames")):
    raises(fls.zvs_frames, bad, "bad.zvs", contains=(text,))

# ----------------------------------------------------------- TOML out

TABLE = {"format": "x", "version": 1, "flag": False, "rate": 100.0,
         "tiny": 1e-06, "big": 1e300, "path": "a \"quoted\" \\ path",
         "list": [1.0, 2.5, -3.0], "ints": [1, 2],
         "control": {"mode": "position", "rate_hz": 100.0},
         "fel": {"gain": 0.5, "regressors": {"smooth_sigma_s": 0.04}},
         "outer": {"inner": {"deep": True}},
         "empty": {}}
text = fls.toml_text(TABLE, "line one\n\nline three")
assert text.startswith("# line one\n#\n# line three\nformat = \"x\"\n"), text
assert tomllib.loads(text) == TABLE
assert "[fel.regressors]" in text and "[outer.inner]" in text
assert "[outer]\n" not in text  # a table of tables only needs no header
assert "[empty]" in text
for value in (float("nan"), float("inf"), -float("inf")):
    loaded = tomllib.loads("v = " + fls.toml_value(value))["v"]
    assert (math.isnan(loaded) if math.isnan(value) else loaded == value)
raises(fls.toml_value, [{"a": 1}], contains=("arrays of tables",))
raises(fls.toml_value, None, contains=("NoneType",))
assert fls.toml_value(True) == "true" and fls.toml_value(3) == "3"

# ------------------------------------------------- config derivation

CONFIG = ROOT / "config"
CONFIG.mkdir()
(CONFIG / "model.ztk").write_text("model\n")
(CONFIG / "hw.toml").write_text("hardware\n")
(CONFIG / "force.toml").write_text("force\n")
(CONFIG / "follow.toml").write_text('''format = "rtctrl-x7-follow"
version = 1
model = "model.ztk"
reference = "absent.zvs"
hardware_config = "hw.toml"

[control]
rate_hz = 100.0
mode = "position"

[simulation]
integration_step_s = 0.0001
position_kp = [20.0, 60.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
position_kd = 0.8
command_delay_cycles = 1
quantize_encoder = true

[finalization]
simulation_hold_time_s = 0.6
''')
(CONFIG / "learn.toml").write_text('''format = "rtctrl-x7-follow-learn"
version = 1

[ilc]
phi = 0.0
advance_s = 0.05

[fel]
step = "normalised-gradient"
gain = 0.2
''')
(CONFIG / "with_offset.toml").write_text(
    (CONFIG / "follow.toml").read_text()
    + '\n[learning]\noffset = "o.csv"\n')

assert fls.deep_merge({"a": {"b": 1, "c": 2}, "d": [1]},
                      {"a": {"c": 3}, "d": [2], "e": 4}) == {
    "a": {"b": 1, "c": 3}, "d": [2], "e": 4}
base = {"a": {"b": 1}}
fls.deep_merge(base, {"a": {"b": 2}})
assert base == {"a": {"b": 1}}  # the inputs are not changed

case = {"name": "X", "follow": str(CONFIG / "follow.toml"),
        "learn": str(CONFIG / "learn.toml"),
        "follow_set": {"simulation": {"command_delay_cycles": 3,
                                      "link_force": str(CONFIG /
                                                        "force.toml")}},
        "follow_scale": {"simulation": {"position_kp": 0.5,
                                        "integration_step_s": 2.0}},
        "learn_set": {"ilc": {"phi": 0.5}, "fel": {"gain": 0.4}}}
derived = fls.derive_follow(case, "/elsewhere/ref.zvs")
assert derived["model"] == str((CONFIG / "model.ztk").resolve())
assert derived["hardware_config"] == str((CONFIG / "hw.toml").resolve())
assert derived["reference"] == "/elsewhere/ref.zvs"
assert derived["control"] == {"rate_hz": 100.0, "mode": "position"}
assert derived["simulation"] == {
    "integration_step_s": 0.0002,
    "position_kp": [10.0, 30.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
    "position_kd": 0.8, "command_delay_cycles": 3, "quantize_encoder": True,
    "link_force": str(CONFIG / "force.toml")}
# a trial's perturbation merges on top
derived = fls.derive_follow(case, "/r.zvs", {"simulation": {
    "quantize_encoder": False}}, {"simulation": {"position_kd": 2.0}})
assert derived["simulation"]["quantize_encoder"] is False
assert derived["simulation"]["position_kd"] == 1.6
assert derived["simulation"]["position_kp"][1] == 30.0
learn = fls.derive_learn(case)
assert learn["ilc"] == {"phi": 0.5, "advance_s": 0.05}
assert learn["fel"] == {"step": "normalised-gradient", "gain": 0.4}
# written and read back unchanged
assert tomllib.loads(fls.toml_text(learn, "h")) == learn
for scale, text in (({"simulation": {"command_delay_cycles": 2.0}},
                     "not a float"),
                    ({"simulation": {"quantize_encoder": 2.0}},
                     "not a float"),
                    ({"simulation": {"absent": 2.0}}, "not in the config")):
    bad = dict(case, follow_scale=scale)
    raises(fls.derive_follow, bad, "/r.zvs", contains=(text,))
raises(fls.derive_follow, dict(case, follow=str(CONFIG / "with_offset.toml")),
       "/r.zvs", contains=("[learning] offset",))
raises(fls.derive_follow, dict(case, follow=str(CONFIG / "absent.toml")),
       "/r.zvs", contains=("cannot read",))

# ------------------------------------------------------------- plans

PLANS = ROOT / "plans"
PLANS.mkdir()
(PLANS / "toy_plan.toml").write_text("[toy]\nduration_s = 1.5\n")
(PLANS / "prepared.zvs").write_text(ZVS)
HEAD = '''format = "rtctrl-learn-study-plan"
version = 1

[references]
toy = { plan = "toy_plan.toml" }
moving = { from = "toy", cut_leading_s = 0.3 }
prepared = { zvs = "prepared.zvs" }

[defaults]
reference = "toy"
follow = "../config/follow.toml"
learn = "../config/learn.toml"
trials = 4
method = "ilc"
learn_set = { ilc = { phi = 0.5 } }
'''
CRITERIA = '''
[criteria]
basic = [
  { id = "a", check = "runs_ok" },
  { id = "b", check = "decreasing" },
  { id = "c", check = "ratio", trial = 3, of = 0, at_most = 0.5 },
  { id = "d", check = "no_clamp" },
  { id = "e", check = "hf_ratio", trial = 3, of = 0, at_most = 1.5 },
]
'''


def plan_file(name, body):
    path = PLANS / name
    path.write_text(body)
    return path


plan = fls.load_plan(plan_file("parse.toml", HEAD + CRITERIA + '''
[[case]]
name = "N1"
note = "nominal"
criteria = "basic"

[[case]]
name = "R1"
like = "N1"
follow_scale = { simulation = { position_kp = 0.5 } }
learn_set = { ilc = { cutoff_hz = 3.0 } }
criteria = [{ id = "a", check = "runs_ok" }]

[[case]]
name = "R1b"
like = "R1"
reference = "moving"
trials = 6
perturb = [{ trials = [2, 4], follow_set = { simulation = { link_force = "../config/force.toml" } } }]

[[case]]
name = "F1"
method = "fel"
learn_set = { fel = { step = "least-squares", gain = 1.0 } }
transfer_to = "prepared"
criteria = [{ id = "t", check = "transfer", at_most = 0.5 },
            { id = "v", check = "vs_case", case = "R1b", trials = [3], at_most = 2.0 }]

[[sweep]]
name = "cross"
like = "N1"
trials = 3
vary = { "learn.ilc.phi" = [0.25, 1.0], "follow.simulation.command_delay_cycles" = [0, 2] }

[[sweep]]
name = "phi"
vary = { "learn.ilc.phi" = [0.75] }
'''))
names = [c["name"] for c in plan["cases"]]
assert names == ["N1", "R1", "R1b", "F1",
                 "cross[phi=0.25,command_delay_cycles=0]",
                 "cross[phi=0.25,command_delay_cycles=2]",
                 "cross[phi=1.0,command_delay_cycles=0]",
                 "cross[phi=1.0,command_delay_cycles=2]", "phi=0.75"], names
cases = {c["name"]: c for c in plan["cases"]}
assert cases["cross[phi=1.0,command_delay_cycles=2]"]["dir"] == \
    "cross_phi=1.0_command_delay_cycles=2"
assert plan["references"]["toy"] == {"kind": "plan", "plan": str(
    (PLANS / "toy_plan.toml").resolve())}
assert plan["references"]["moving"] == {"kind": "from", "from": "toy",
                                        "cut_leading_s": 0.3}
assert plan["split_hz"] == 3.0 and len(plan["sha256"]) == 64
n1, r1, r1b, f1 = (cases[n] for n in ("N1", "R1", "R1b", "F1"))
assert n1["follow"] == str((CONFIG / "follow.toml").resolve())
assert n1["method"] == "ilc" and n1["trials"] == 4 and n1["note"] == "nominal"
assert [c["id"] for c in n1["criteria"]] == ["a", "b", "c", "d", "e"]
assert n1["learn_set"] == {"ilc": {"phi": 0.5}}
# like: the parent's settings, the child's merged in; criteria not inherited
assert r1["learn_set"] == {"ilc": {"phi": 0.5, "cutoff_hz": 3.0}}
assert r1["follow_scale"] == {"simulation": {"position_kp": 0.5}}
assert r1b["follow_scale"] == r1["follow_scale"] and r1b["criteria"] == []
assert r1b["reference"] == "moving" and r1b["trials"] == 6
assert r1b["perturb"] == [{"trials": [2, 4], "follow_set": {"simulation": {
    "link_force": str((CONFIG / "force.toml").resolve())}},
    "follow_scale": {}}]
assert f1["learn_set"] == {"ilc": {"phi": 0.5}, "fel": {
    "step": "least-squares", "gain": 1.0}}
assert f1["transfer_to"] == "prepared" and n1["transfer_to"] is None
sweep = cases["cross[phi=1.0,command_delay_cycles=2]"]
assert sweep["kind"] == "sweep" and sweep["criteria"] == []
assert sweep["trials"] == 3 and sweep["learn_set"] == {"ilc": {"phi": 1.0}}
assert sweep["follow_set"] == {"simulation": {"command_delay_cycles": 2}}
assert sweep["vary"] == {"learn.ilc.phi": 1.0,
                         "follow.simulation.command_delay_cycles": 2}
assert cases["phi=0.75"]["learn_set"] == {"ilc": {"phi": 0.75}}

# Every refusal names the problem; nothing is guessed.
REFUSALS = [
    ('format = "other"\nversion = 1\n', "expected format"),
    (HEAD + "bogus = 1\n", "unknown key(s) bogus"),
    (HEAD.replace('trials = 4', 'trials = 0') + '[[case]]\nname = "A"\n',
     "trials must be an integer >= 1"),
    (HEAD.replace('trials = 4', 'trials = true') + '[[case]]\nname = "A"\n',
     "trials must be an integer >= 1"),
    (HEAD.replace('method = "ilc"', 'method = "pid"')
     + '[[case]]\nname = "A"\n', "method must be one of"),
    (HEAD, "no cases"),
    (HEAD + '[[case]]\nname = "A"\nreference = "nope"\n',
     "reference 'nope' is not in [references]"),
    (HEAD + '[[case]]\nname = "A"\nfollow = "absent.toml"\n',
     "no such file"),
    (HEAD + '[[case]]\nname = "A"\n[[case]]\nname = "A"\n',
     "case name 'A' is not unique"),
    (HEAD + '[[case]]\nname = "a b"\n', "name must match"),
    (HEAD + '[[case]]\nname = "A"\nlike = "B"\n',
     "like 'B' must name an earlier case"),
    (HEAD + '[[case]]\nname = "A"\nwhatever = 1\n', "unknown key(s)"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = "none"\n',
     "no criteria set 'none'"),
    (HEAD + CRITERIA + '[[case]]\nname = "A"\ncriteria = [{ id = "a", '
     'check = "fast" }]\n', "unknown check 'fast'"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = [{ id = "a", check = '
     '"ratio", trial = 3, of = 0 }]\n', "ratio needs at_most"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = [{ id = "a", check = '
     '"ratio", trial = 4, of = 0, at_most = 0.5 }]\n', "trial 4 is not run"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = [{ id = "a", check = '
     '"ratio", trial = 3, of = 0, at_most = 0 }]\n', "at_most must be"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = [{ id = "a", check = '
     '"runs_ok", extra = 1 }]\n', "unknown key(s) extra"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = [{ id = "a", check = '
     '"runs_ok" }, { id = "a", check = "no_clamp" }]\n', "unique"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = [{ id = "a", check = '
     '"vs_case", case = "Z", trials = [1], at_most = 1.0 }]\n', "no case 'Z'"),
    (HEAD + '[[case]]\nname = "A"\ntrials = 2\n[[case]]\nname = "B"\n'
     'criteria = [{ id = "a", check = "vs_case", case = "A", trials = [3], '
     'at_most = 1.0 }]\n', "case A does not run trial 3"),
    (HEAD + '[[case]]\nname = "A"\ncriteria = [{ id = "a", check = '
     '"transfer", at_most = 0.5 }]\n', "transfer needs transfer_to"),
    (HEAD + '[[case]]\nname = "A"\ntransfer_to = "prepared"\n',
     "transfer_to needs method fel"),
    (HEAD + '[[case]]\nname = "A"\nfollow_set = { reference = "x.zvs" }\n',
     "'reference' is set by the study"),
    (HEAD + '[[case]]\nname = "A"\nfollow_set = { learning = { offset = '
     '"o.csv" } }\n', "'learning.offset' is set by the study"),
    (HEAD + '[[case]]\nname = "A"\nfollow_scale = { simulation = { '
     'position_kp = -1.0 } }\n', "must be a finite number > 0"),
    (HEAD + '[[case]]\nname = "A"\nfollow_set = { simulation = { '
     'position_kp = 1.0 } }\nfollow_scale = { simulation = { '
     'position_kp = 2.0 } }\n', "both set and scaled"),
    (HEAD + '[[case]]\nname = "A"\nperturb = [{ trials = [4] }]\n',
     "perturbed trial 4 is not run"),
    (HEAD + '[[case]]\nname = "A"\nperturb = [{ trials = [] }]\n',
     "non-empty array of trial indices"),
    (HEAD + '[[case]]\nname = "A"\nperturb = [{ trials = [1], x = 1 }]\n',
     "unknown key(s) x"),
    (HEAD + '[[sweep]]\nname = "s"\nvary = { "ilc.phi" = [1.0] }\n',
     "must start with 'learn.' or 'follow.'"),
    (HEAD + '[[sweep]]\nname = "s"\nvary = { "learn.ilc.phi" = [] }\n',
     "needs an array of values"),
    (HEAD + '[[sweep]]\nname = "s"\nvary = {}\n', "vary is empty"),
    (HEAD + '[[sweep]]\nname = "s"\ncriteria = "x"\n'
     'vary = { "learn.ilc.phi" = [1.0] }\n', "unknown key(s) criteria"),
    (HEAD.replace('moving = { from = "toy", cut_leading_s = 0.3 }',
                  'moving = { from = "later", cut_leading_s = 0.3 }'),
     "must name an earlier reference"),
    (HEAD.replace('cut_leading_s = 0.3', 'cut_leading_s = 0.3, '
                  'cut_leading_frames = 3'), "exactly one of cut_leading_s"),
    (HEAD.replace('cut_leading_s = 0.3', 'cut_leading_frames = 0'),
     "cut_leading_frames must be >= 1"),
    (HEAD.replace('prepared = { zvs = "prepared.zvs" }',
                  'prepared = { zvs = "prepared.zvs", plan = "p.toml" }'),
     "exactly one of plan, zvs, from"),
    (HEAD.replace('prepared.zvs', 'absent.zvs'), "no such file"),
    (HEAD.replace('version = 1\n', 'version = 1\nsplit_hz = 0\n', 1),
     "split_hz must be a number > 0"),
    ("format = [", "cannot read the plan"),
]
for i, (body, text) in enumerate(REFUSALS):
    raises(fls.load_plan, plan_file(f"refused_{i}.toml", body),
           contains=(text,))

# -------------------------------------------------------------- checks


def measured(rms, hf=None, updates=None, completed=True, planned=None,
             transfer=None, stop=None):
    planned = len(rms) if planned is None else planned
    if updates is None:
        updates = [{"trial": k, "step_clamped": 0, "abs_clamped": 0}
                   for k in range(planned - 1)]
    return {"planned_trials": planned, "rms": rms,
            "hf": hf if hf is not None else [1e-3] * len(rms),
            "updates": updates, "completed": completed, "stop": stop,
            "transfer": transfer}


def verdict(check, own, others=None, **params):
    return fls.evaluate({"id": "x", "check": check, **params}, own,
                        others or {})


CURVE = [0.020, 0.010, 0.005, 0.003]
assert verdict("runs_ok", measured(CURVE))["verdict"] == "PASS"
result = verdict("runs_ok", measured(CURVE[:2], planned=4, completed=False,
                                     stop="update after trial 1 refused"))
assert result["verdict"] == "FAIL" and "update after trial 1" in \
    result["detail"]
assert verdict("decreasing", measured(CURVE))["verdict"] == "PASS"
# strict: an equal value is not a decrease
result = verdict("decreasing", measured([0.02, 0.01, 0.01, 0.005]))
assert result["verdict"] == "FAIL" and "trial(s) 2 (10.0 after 10.0" in \
    result["detail"], result
assert verdict("decreasing", measured([0.02, 0.01, 0.01, 0.005]),
               **{"from": 2})["verdict"] == "PASS"
assert verdict("decreasing", measured([0.02, 0.01, 0.03]), to=1)[
    "verdict"] == "PASS"
result = verdict("decreasing", measured([0.02, 0.01], planned=4,
                                        completed=False, stop="sim_2 failed"))
assert result["verdict"] == "FAIL"
assert result["detail"].startswith("not evaluable: RMS of trial 2 not "
                                   "measured"), result
assert "case stopped: sim_2 failed" in result["detail"]
# ratio and hf_ratio: inclusive bounds
assert verdict("ratio", measured(CURVE), trial=3, of=0, at_most=0.15)[
    "verdict"] == "PASS"
result = verdict("ratio", measured(CURVE), trial=3, of=0, at_most=0.149)
assert result["verdict"] == "FAIL" and "= 0.1500" in result["detail"]
assert verdict("ratio", measured([0.02, 0.003]), trial=1, of=0,
               at_most=0.15)["verdict"] == "PASS"
hf = [0.001, 0.0012, 0.0015, 0.00151]
assert verdict("hf_ratio", measured(CURVE, hf=hf), trial=2, of=0,
               at_most=1.5)["verdict"] == "PASS"
assert verdict("hf_ratio", measured(CURVE, hf=hf), trial=3, of=0,
               at_most=1.5)["verdict"] == "FAIL"
result = verdict("bounded", measured([0.02, 0.01, 0.012, 0.0121]),
                 trials=[2, 3], of=1, at_most=1.2)
assert result["verdict"] == "FAIL" and "above at trial(s) [3]" in \
    result["detail"]
assert verdict("bounded", measured([0.02, 0.01, 0.012, 0.012]),
               trials=[2, 3], of=1, at_most=1.2)["verdict"] == "PASS"
clamped = [{"trial": 0, "step_clamped": 0, "abs_clamped": 0},
           {"trial": 1, "step_clamped": 7, "abs_clamped": 2},
           {"trial": 2, "step_clamped": 0, "abs_clamped": 0}]
assert verdict("no_clamp", measured(CURVE))["verdict"] == "PASS"
result = verdict("no_clamp", measured(CURVE, updates=clamped))
assert result["verdict"] == "FAIL" and "step clamp 7, value clamp 2" in \
    result["detail"]
result = verdict("no_clamp", measured(CURVE, updates=clamped[:1]))
assert result["verdict"] == "FAIL" and "1 of 3 updates ran" in \
    result["detail"]
others = {"REF": measured([0.02, 0.01, 0.006, 0.004])}
assert verdict("vs_case", measured(CURVE), others, case="REF",
               trials=[2, 3], at_most=1.0)["verdict"] == "PASS"
result = verdict("vs_case", measured([0.02, 0.01, 0.0091, 0.004]), others,
                 case="REF", trials=[2, 3], at_most=1.5)
assert result["verdict"] == "FAIL", result
assert "against REF (at most 1.5): 2: 9.10/6.00 = 1.517, 3: 4.00/4.00 = " \
    "1.000" == result["detail"], result
result = verdict("vs_case", measured(CURVE), {}, case="REF", trials=[2],
                 at_most=1.0)
assert result["verdict"] == "FAIL" and "case REF not run" in result["detail"]
done = {"zero_rms_rad": 0.02, "synth_rms_rad": 0.01}
assert verdict("transfer", measured(CURVE, transfer=done), at_most=0.5)[
    "verdict"] == "PASS"
assert verdict("transfer", measured(CURVE, transfer=done), at_most=0.49)[
    "verdict"] == "FAIL"
assert "not evaluable" in verdict("transfer", measured(CURVE),
                                  at_most=0.5)["detail"]

# ---------------------------------------------------- offset statistics

offset = ROOT / "offset.csv"
offset.write_text("# format: rtctrl-x7-follow-offset\n# version: 1\n"
                  "# method: ilc\n# trial: 2\n# dt_s: 0.01\n# samples: 3\n"
                  "# reference_duration_s: 0.02\n# reference_sha256: "
                  + "a" * 64 + "\n# parent_sha256: " + "b" * 64
                  + "\n# source_log_sha256:\n"
                  "time_s,delta0_rad,delta1_rad,delta2_rad,delta3_rad,"
                  "delta4_rad,delta5_rad,delta6_rad,delta7_rad\n"
                  "0.000000000,0,0.001,0,0,0,0,0,0\n"
                  "0.010000000,0,-0.002,0,0,0,0,0,0\n"
                  "0.020000000,0.0005,-0.002,0,0,0,0,0,0\n")
stats = fls.parse_offset(offset)
assert stats["method"] == "ilc" and stats["trial"] == 2
assert stats["samples"] == 3 and stats["source_log_sha256"] == ""
assert stats["max_abs_rad"] == 0.002
assert abs(stats["max_rate_rad_s"] - 0.3) < 1e-12
assert stats["max_abs_rad_by_joint"][:2] == [0.0005, 0.002]
assert stats["parent_sha256"] == "b" * 64

# ------------------------------------------------------------ summary

summary_path = ROOT / "summary.json"
fls.write_summary(summary_path, {"a": float("nan"), "b": [1.5, float("inf")],
                                 "c": pathlib.Path("/x"), "d": (1, 2)})
assert json.loads(summary_path.read_text()) == {"a": None, "b": [1.5, None],
                                                "c": "/x", "d": [1, 2]}
raises(fls.write_summary, summary_path, {}, error=FileExistsError)

# ----------------------------------------------- stand-in binaries

FAKE_TOOLS = r'''
"""Stand-ins for x7_plan_ptp, x7_follow_sim and x7_follow_learn: the same
arguments and files, a first-order lag plant with a static sag."""
import hashlib
import math
import os
import sys
import tomllib

import numpy as np

DOF = 8


def note_call(name):
    with open(os.environ["FAKE_CALLS"], "a") as stream:
        stream.write(name + " " + " ".join(sys.argv[1:]) + "\n")


def arg(flag):
    args = sys.argv
    return args[args.index(flag) + 1] if flag in args else None


def sha(path):
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def load(path):
    with open(path, "rb") as stream:
        return tomllib.load(stream)


def new_file(path, text):
    with open(path, "x") as stream:
        stream.write(text)


def reference(config):
    """Grid times and qref (N x 8) of a follow config's reference."""
    frames = []
    with open(config["reference"]) as stream:
        for line in stream:
            dt, rest = line.split(" ", 1)
            values = rest.split("(")[1].split(")")[0].split()
            frames.append((float(dt), [float(v) for v in values][:DOF]))
    times = np.cumsum([0.0] + [dt for dt, _ in frames[:-1]])
    duration = sum(dt for dt, _ in frames)
    rate = config["control"]["rate_hz"]
    intervals = max(1, math.ceil(duration * rate - 1e-9))
    grid = np.arange(intervals + 1) * (duration / intervals)
    q = np.array([v for _, v in frames])
    qref = np.column_stack([np.interp(grid, times, q[:, j])
                            for j in range(DOF)])
    return grid, qref, duration


def signature(config):
    return hashlib.sha256((sha(config["reference"]) + ":"
                           + repr(config["control"]["rate_hz"])).encode()
                          ).hexdigest()


def read_offset(path):
    head, rows = {}, []
    with open(path) as stream:
        for line in stream:
            if line.startswith("# "):
                key, _, value = line[2:].rstrip("\n").partition(":")
                head[key] = value.strip()
            elif not line.startswith("time_s"):
                rows.append([float(v) for v in line.split(",")[1:]])
    return head, np.array(rows)


def write_offset(path, config, delta, method, trial, parent, source):
    grid, _, duration = reference(config)
    text = ("# format: rtctrl-x7-follow-offset\n# version: 1\n"
            f"# method: {method}\n# trial: {trial}\n"
            f"# dt_s: {float(grid[1] - grid[0])!r}\n# samples: {len(grid)}\n"
            f"# reference_duration_s: {duration!r}\n"
            f"# reference_sha256: {signature(config)}\n"
            f"# parent_sha256:{' ' + parent if parent else ''}\n"
            f"# source_log_sha256:{' ' + source if source else ''}\n"
            "time_s," + ",".join(f"delta{j}_rad" for j in range(DOF)) + "\n")
    for t, row in zip(grid, delta):
        text += f"{t:.9f}," + ",".join(repr(float(v)) for v in row) + "\n"
    new_file(path, text)


def plan_ptp():
    note_call("x7_plan_ptp")
    toy = load(arg("--config"))["toy"]
    n = int(round(toy["duration_s"] / 0.01)) + 1
    lines = []
    for i in range(n):
        s = i / (n - 1)
        shape = s ** 3 * (10 - 15 * s + 6 * s * s)
        values = [0.2 * (j + 1) * shape * (-1) ** j for j in range(7)]
        values += [0.0, 0.0]
        lines.append("0.01 9 (" + " ".join(repr(v) for v in values) + " )\n")
    new_file(arg("--output"), "".join(lines))
    if arg("--diagnostics"):
        new_file(arg("--diagnostics"), "frame\n")
    print(f"wrote {n} frames to {arg('--output')}")


HEADER = ["schema_version", "time_s", "phase", "command_mode",
          "phase_time_s", "feedback_time_s", "feedback_seq",
          "receipt_accepted", "submitted_seq", "applied_seq"]
for _j in range(DOF):
    HEADER += [f"qref{_j}_rad", f"dqref{_j}_rad_s", f"ddqref{_j}_rad_s2",
               f"q{_j}_rad", f"dq{_j}_rad_s", f"tau{_j}_nm",
               f"qcmd{_j}_rad", f"dqcmd{_j}_rad_s", f"effort_limit{_j}_nm",
               f"error{_j}_rad", f"applied{_j}",
               f"applied_effort_limit{_j}_nm", f"flags{_j}"]
HOLD = 60


def follow_sim():
    note_call("x7_follow_sim")
    config = load(arg("--config"))
    offset = arg("--offset")
    head, delta = read_offset(offset)
    if int(head["trial"]) == int(os.environ.get("FAKE_SIM_FAIL_TRIAL", -1)):
        sys.exit("x7_follow_sim: simulated failure")
    print(f"command offset accepted: method {head['method']}, trial "
          f"{head['trial']}, max |delta| {np.max(np.abs(delta)):g} rad, "
          f"sha256 {sha(offset)}")
    grid, qref, _ = reference(config)
    h = grid[1] - grid[0]
    sim = config.get("simulation", {})
    kp = sim.get("position_kp", 20.0)
    kp = kp[0] if isinstance(kp, list) else kp
    a = math.exp(-h / (0.05 * 20.0 / kp))
    n = len(grid)
    qref_all = np.vstack([qref, np.repeat(qref[-1:], HOLD, axis=0)])
    u = qref_all + np.vstack([delta, np.repeat(delta[-1:], HOLD, axis=0)])
    q = np.zeros_like(u)
    q[0] = u[0]
    for i in range(1, len(u)):
        q[i] = a * q[i - 1] + (1 - a) * u[i - 1]
    q[:, :7] -= 0.002  # a static sag the offset has to learn
    if "link_force" in sim:  # a bump in the middle of tracking
        bump = np.exp(-((np.arange(len(u)) - n // 2) * h / 0.2) ** 2)
        q[:, :7] += 0.05 * bump[:, None]
    q[:, 7] = 0.0
    q = np.round(q / 1e-5) * 1e-5  # a coarse encoder
    rows, seq = [], 0

    def row(phase, t, t0, k, i):
        nonlocal seq
        seq += 1
        fields = ["1", f"{t:.9f}", phase, "3", f"{t - t0:.9f}", f"{t:.9f}",
                  str(seq), "1", str(seq), str(seq)]
        for j in range(DOF):
            dq = (qref_all[min(i + 1, len(u) - 1), j]
                  - qref_all[max(i - 1, 0), j]) / (2 * h)
            fields += [f"{qref_all[i, j]:.9g}", f"{dq:.9g}", "0",
                       f"{q[i, j]:.9g}", "0", f"{kp * (u[i, j] - q[i, j]):.9g}",
                       f"{u[i, j]:.9g}", "0", "1",
                       f"{q[i, j] - qref_all[i, j]:.9g}", f"{u[i, j]:.9g}",
                       "1", "0"]
        rows.append(",".join(fields))

    for k in range(3):
        row("home", -0.05 + 0.01 * k, -0.05, k, 0)
    for k in range(2):
        row("offset_ramp", -0.02 + 0.01 * k, -0.02, k, 0)
    for i in range(n):
        row("tracking", grid[i], 0.0, i, i)
    for i in range(HOLD):
        row("finalizing", grid[-1] + (i + 1) * h, grid[-1], i, n + i)
    new_file(arg("--log"), ",".join(HEADER) + "\n" + "\n".join(rows) + "\n")
    new_file(arg("--motion"), "0.01 9 (0 0 0 0 0 0 0 0 0 )\n")
    worst = float(np.max(np.abs(q[:n] - qref)))
    print(f"follow simulation: success, {len(rows)} cycles, home 0.0000 rad,"
          f" tracking {worst:.4f} rad")
    print(f"motion: {arg('--motion')}\nlog: {arg('--log')}")


def read_log(path):
    import csv
    with open(path, newline="") as stream:
        data = list(csv.DictReader(stream))
    track = [r for r in data if r["phase"] == "tracking"]
    tail = [r for r in data if r["phase"] == "finalizing"]

    def error(rows):
        return np.array([[float(r[f"qref{j}_rad"]) - float(r[f"q{j}_rad"])
                          for j in range(DOF)] for r in rows])

    return error(track), error(tail)


def report_text(k, e, delta, extra):
    rms = np.sqrt(np.mean(e ** 2, axis=0))
    out = (f'format = "rtctrl-x7-follow-learn-report"\nversion = 1\n'
           f'measured_trial = {k}\noutput_trial = {k + 1}\n[tracking]\n'
           f'aggregate_rms_rad = {float(np.sqrt(np.mean(rms ** 2)))!r}\n'
           f'arm_rms_rad = {float(np.sqrt(np.mean(rms[:7] ** 2)))!r}\n'
           f'error_rms_rad = {[float(v) for v in rms]}\n[offset]\n'
           f'rms_rad = {[float(v) for v in np.sqrt(np.mean(delta ** 2, 0))]}\n'
           f'max_abs_rad = {[float(v) for v in np.max(np.abs(delta), 0)]}\n')
    return out + extra


def regressors(config):
    _, qref, _ = reference(config)
    return qref - qref[0]


def synthesis(config, weights):
    shape = regressors(config)
    delta = np.zeros_like(shape)
    for j in range(7):
        delta[:, j] = weights["w0"][j] + weights["w1"][j] * shape[:, j]
    return delta


def weights_text(trial, w0, w1):
    return (f"trial = {trial}\nw0 = {[float(v) for v in w0]}\n"
            f"w1 = {[float(v) for v in w1]}\n")


def follow_learn():
    command = sys.argv[1]
    note_call("x7_follow_learn " + command)
    if command == "init-weights":
        new_file(arg("--output"), weights_text(0, [0.0] * 7, [0.0] * 7))
        return
    follow_path = arg("--follow-config")
    config = load(follow_path)
    if command == "init":
        grid, _, _ = reference(config)
        write_offset(arg("--output"), config, np.zeros((len(grid), DOF)),
                     "zero", 0, "", "")
        return
    learn = load(arg("--config"))
    if command == "synth":
        weights = load(arg("--weights"))
        write_offset(arg("--output"), config, synthesis(config, weights),
                     "fel", weights["trial"], sha(arg("--weights")), "")
        return
    head, delta = read_offset(arg("--offset"))
    k = int(head["trial"])
    e, tail = read_log(arg("--log"))
    grid, _, _ = reference(config)
    h = grid[1] - grid[0]
    section = learn.get(command, {})
    shift = int(round(section.get("advance_s", 0.05) / h))
    ahead = np.vstack([e, tail])[shift:shift + len(e)]
    if command == "ilc":
        phi = learn["ilc"]["phi"]
        if phi >= 1.5:
            sys.exit("x7_follow_learn: the next offset exceeds "
                     "limits.max_offset_rate_rad_s at joint 0, sample 3; "
                     "lower [ilc] cutoff_hz or the gains")
        new = delta + phi * ahead
        new[:, 7] = 0.0
        clamped = [3 if phi > 1.0 else 0] + [0] * 7
        extra = (f"[ilc]\nstep_clamped = {clamped}\n"
                 f"abs_clamped = {[0] * 8}\n")
        write_offset(arg("--output"), config, new, "ilc", k + 1,
                     sha(arg("--offset")), sha(arg("--log")))
    else:
        gain = learn["fel"]["gain"]
        weights = load(arg("--weights"))
        shape = regressors(config)
        w0, w1 = list(weights["w0"]), list(weights["w1"])
        for j in range(7):
            x = np.column_stack([np.ones(len(e)), shape[:, j]])
            step = np.linalg.lstsq(x, ahead[:, j], rcond=None)[0]
            w0[j] += gain * float(step[0])
            w1[j] += gain * float(step[1])
        new_file(arg("--output-weights"), weights_text(k + 1, w0, w1))
        new = synthesis(config, {"w0": w0, "w1": w1})
        extra = ("[fel]\nkept_columns = [2]\nskipped = [[]]\n"
                 "lambda_max = [1.0]\nlambda_min = [1.0]\n"
                 "s_rms_rad = [0.0]\nresidual_rms_rad = [0.0]\n")
        write_offset(arg("--output"), config, new, "fel", k + 1,
                     sha(arg("--output-weights")), sha(arg("--log")))
    new_file(arg("--report"), report_text(k, e, new, extra))
'''
BIN = ROOT / "bin"
BIN.mkdir()
(BIN / "fake_tools.py").write_text(FAKE_TOOLS)
for binary, entry in (("x7_plan_ptp", "plan_ptp"),
                      ("x7_follow_sim", "follow_sim"),
                      ("x7_follow_learn", "follow_learn")):
    path = BIN / binary
    path.write_text(f"#!{sys.executable}\nimport sys\n"
                    f"sys.path.insert(0, {str(BIN)!r})\n"
                    f"import fake_tools\nfake_tools.{entry}()\n")
    path.chmod(0o755)
CALLS = ROOT / "calls.txt"
os.environ["FAKE_CALLS"] = str(CALLS)

# The study refuses any other program before it runs anything.
tools = fls.Tools(BIN, ROOT)
for name in ("x7_follow", "x7_read", "dxl_inspect", "/bin/sh"):
    raises(tools.run, "step", name, [], contains=("refusing to run",))
assert not CALLS.exists()

E2E = plan_file("e2e.toml", HEAD + CRITERIA + '''
[[case]]
name = "N1"
criteria = "basic"

[[case]]
name = "M1"
like = "N1"
reference = "moving"
trials = 3
criteria = [{ id = "a", check = "runs_ok" }]

[[case]]
name = "D1"
like = "N1"
perturb = [{ trials = [1], follow_set = { simulation = { link_force = "../config/force.toml" } } }]
criteria = [{ id = "a", check = "runs_ok" },
            { id = "b", check = "vs_case", case = "N1", trials = [2, 3], at_most = 2.0 },
            { id = "c", check = "bounded", trials = [2, 3], of = 0, at_most = 1.0 }]

[[case]]
name = "S1"
like = "N1"
learn_set = { ilc = { phi = 1.6 } }
criteria = [{ id = "a", check = "runs_ok" }, { id = "b", check = "decreasing" }]

[[case]]
name = "C1"
like = "N1"
trials = 2
learn_set = { ilc = { phi = 1.2 } }
criteria = [{ id = "d", check = "no_clamp" }]

[[case]]
name = "F1"
method = "fel"
trials = 3
learn_set = { fel = { gain = 1.0 } }
transfer_to = "moving"
criteria = [{ id = "a", check = "runs_ok" }, { id = "t", check = "transfer", at_most = 0.5 }]

[[sweep]]
name = "phi"
like = "N1"
trials = 2
vary = { "learn.ilc.phi" = [0.25, 0.75] }
''')
OUT = ROOT / "out_sequential"
status, out, err = run_main(E2E, "--out", OUT, "--bin-dir", BIN)
assert status == 1, (status, out, err)  # S1 and C1 fail on purpose
summary = json.loads((OUT / "summary.json").read_text())
assert summary["format"] == "rtctrl-learn-study-summary"
assert summary["version"] == 1 and summary["jobs"] == 1
assert summary["plan_sha256"] == fls.sha256_file(E2E)
assert (OUT / "plan.toml").read_bytes() == E2E.read_bytes()
assert set(summary["binaries"]) == {"x7_plan_ptp", "x7_follow_sim",
                                    "x7_follow_learn"}
results = {c["name"]: c for c in summary["cases"]}
assert list(results) == ["N1", "M1", "D1", "S1", "C1", "F1", "phi=0.25",
                         "phi=0.75"]

# The references: planned, cut (whole frames, the rest byte for byte),
# copied.
references = summary["references"]
toy_text = (OUT / "references" / "toy.zvs").read_text()
assert references["toy"]["frames"] == 151
assert references["moving"]["frames_cut"] == 30
assert abs(references["moving"]["time_cut_s"] - 0.3) < 1e-12
assert (OUT / "references" / "moving.zvs").read_text() == "".join(
    toy_text.splitlines(keepends=True)[30:])
assert (OUT / "references" / "prepared.zvs").read_text() == ZVS
assert references["prepared"]["sha256"] == fls.sha256_file(
    PLANS / "prepared.zvs")

# The converging case: every trial and update ran, the curve falls, and
# its criteria pass. The last trial's RMS comes from its log, the others
# from the update's report, which agree with the logs.
n1 = results["N1"]
assert n1["status"] == "completed" and n1["stop"] is None, n1["stop"]
assert [c["verdict"] for c in n1["criteria"]] == ["PASS"] * 5, n1["criteria"]
assert [t["rms_source"] for t in n1["trials"]] == ["report"] * 3 + ["log"]
for trial in n1["trials"]:
    assert abs(trial["rms_rad"] - trial["log"]["rms_rad"]) < 1e-8
    assert trial["simulation"]["exit"] == 0
    assert trial["simulation"]["result_line"].startswith(
        "follow simulation: success,")
    assert trial["offset"]["trial"] == trial["trial"]
    assert len(trial["log"]["rms_rad_by_joint"]) == 8
    # The above-split RMS is follow_learnability's anatomy aggregate of the
    # trial's log, the RMS over joints of the per-joint values.
    anatomy = fl.anatomy(fl.load_tracking(trial["log"]["path"],
                                          fl.ANATOMY_COLUMNS),
                         summary["split_hz"])
    assert trial["log"]["hf_rms_rad"] == \
        anatomy["aggregate"]["all"]["e_rms_above_split_rad"]
    assert trial["log"]["hf_arm_rms_rad"] == \
        anatomy["aggregate"]["arm"]["e_rms_above_split_rad"]
    by_joint = trial["log"]["hf_rms_rad_by_joint"]
    assert trial["log"]["hf_rms_rad"] == fl.aggregate_rms(by_joint)
    assert trial["log"]["hf_arm_rms_rad"] == fl.aggregate_rms(by_joint[:7])
rms = [t["rms_rad"] for t in n1["trials"]]
assert all(rms[k] < rms[k - 1] for k in range(1, 4)), rms
assert n1["observations"]["monotone"] is True
assert n1["observations"]["trial_of_min"] == 3
case_dir = OUT / "N1"
for name in ("follow.toml", "learn.toml", "offset_0.csv", "offset_3.csv",
             "trial_3.csv", "report_2.toml", "sim_0.out", "update_2.err",
             "case.json"):
    assert (case_dir / name).is_file(), name
assert not (case_dir / "offset_4.csv").exists()   # no update after trial 3
assert not (case_dir / "report_3.toml").exists()
follow = tomllib.loads((case_dir / "follow.toml").read_text())
assert follow["reference"] == references["toy"]["path"]
assert follow["model"] == str((CONFIG / "model.ztk").resolve())
learn = tomllib.loads((case_dir / "learn.toml").read_text())
assert learn["ilc"] == {"phi": 0.5, "advance_s": 0.05}
assert json.loads((case_dir / "case.json").read_text())["status"] == \
    "completed"

# The table: one line per case, verdicts and the curve in mrad.
line = next(text for text in out.splitlines() if text.startswith("N1 "))
assert "a:PASS b:PASS c:PASS d:PASS e:PASS" in line
assert "rms " + " ".join(fls.sig3(v * 1e3) for v in rms) in line, line
assert f"summary: {OUT / 'summary.json'}" in out

# The moving reference starts with a nonzero reference speed.
m1 = results["M1"]
assert m1["status"] == "completed"
assert max(m1["trials"][0]["log"]["dqref_first_abs_rad_s"]) > 0.0
assert max(m1["trials"][0]["log"]["dqref_first_abs_rad_s"]) > 100 * max(
    n1["trials"][0]["log"]["dqref_first_abs_rad_s"])

# The perturbed trial ran with its own configuration, the others did not.
d1 = results["D1"]
assert [t["perturbed"] for t in d1["trials"]] == [False, True, False, False]
perturbed = tomllib.loads((OUT / "D1" / "follow_trial1.toml").read_text())
assert perturbed["simulation"]["link_force"] == str(
    (CONFIG / "force.toml").resolve())
assert "link_force" not in tomllib.loads(
    (OUT / "D1" / "follow.toml").read_text()).get("simulation", {})
assert d1["trials"][1]["follow_config"].endswith("follow_trial1.toml")
assert d1["trials"][0]["rms_rad"] == n1["trials"][0]["rms_rad"]
assert d1["trials"][1]["rms_rad"] > n1["trials"][1]["rms_rad"]
assert [c["verdict"] for c in d1["criteria"]] == ["PASS"] * 3, d1["criteria"]

# A refused update stops its case: recorded verbatim, nothing after it,
# nothing retried. Its criteria fail, saying why.
s1 = results["S1"]
assert s1["status"] == "stopped"
assert s1["stop"]["reason"] == "update after trial 0 refused: exit 1"
assert "exceeds limits.max_offset_rate_rad_s at joint 0, sample 3" in \
    s1["stop"]["stderr"]
assert s1["stop"]["argv"][1] == "ilc"
assert len(s1["trials"]) == 1 and s1["trials"][0]["rms_source"] == "log"
assert [c["verdict"] for c in s1["criteria"]] == ["FAIL", "FAIL"]
assert "update after trial 0 refused" in s1["criteria"][0]["detail"]
assert "not evaluable: RMS of trial 1" in s1["criteria"][1]["detail"]
calls = CALLS.read_text().splitlines()
s1_calls = [c for c in calls if "/S1/" in c]
assert [c.split(" ")[0:2] for c in s1_calls] == [
    ["x7_follow_learn", "init"], ["x7_follow_sim", "--config"],
    ["x7_follow_learn", "ilc"]], s1_calls
assert "STOPPED: update after trial 0 refused" in next(
    text for text in out.splitlines() if text.startswith("S1 "))
assert "FAIL S1 a (runs_ok): stopped: update after trial 0 refused" in out

# Clamp counts come from the update reports.
c1 = results["C1"]
assert c1["criteria"][0]["verdict"] == "FAIL"
assert "step clamp 3, value clamp 0" in c1["criteria"][0]["detail"]

# Feedback-error learning: weights chained, then the transfer runs.
f1 = results["F1"]
assert f1["status"] == "completed", f1["stop"]
assert (OUT / "F1" / "weights_2.toml").is_file()
transfer = f1["transfer"]
assert transfer["reference"] == "moving"
assert transfer["weights"].endswith("weights_2.toml")
assert transfer["runs"]["zero"]["offset"]["trial"] == 0
assert transfer["runs"]["synth"]["offset"]["trial"] == 2
assert transfer["runs"]["synth"]["offset"]["method"] == "fel"
ratio = transfer["synth_rms_rad"] / transfer["zero_rms_rad"]
assert f"= {ratio:.4f}" in f1["criteria"][1]["detail"]
assert f1["criteria"][1]["verdict"] == ("PASS" if ratio <= 0.5 else "FAIL")
fel_calls = [c.split(" ")[1] for c in calls
             if "/F1/" in c and c.startswith("x7_follow_learn")]
assert fel_calls == ["init", "init-weights", "fel", "fel", "init",
                     "synth"], fel_calls

# Sweeps are recorded, with no criteria.
sweep = results["phi=0.75"]
assert sweep["criteria"] == [] and sweep["status"] == "completed"
assert sweep["spec"]["vary"] == {"learn.ilc.phi": 0.75}
assert "(record)  | rms" in next(
    text for text in out.splitlines() if text.startswith("phi=0.75 "))

# Only the three allowed programs ran, each with its own arguments.
assert {c.split(" ")[0] for c in calls} == {"x7_plan_ptp", "x7_follow_sim",
                                            "x7_follow_learn"}

# In parallel processes the cases give the same results.
OUT2 = ROOT / "out_parallel"
status, out2, err2 = run_main(E2E, "--out", OUT2, "--bin-dir", BIN,
                              "--jobs", "3")
assert status == 1, (out2, err2)
parallel = json.loads((OUT2 / "summary.json").read_text())
assert parallel["jobs"] == 3
for case, other in zip(summary["cases"], parallel["cases"]):
    assert case["name"] == other["name"]
    assert [t["rms_rad"] for t in case["trials"]] == \
        [t["rms_rad"] for t in other["trials"]], case["name"]
    assert case["criteria"] == other["criteria"], case["name"]
    assert case["status"] == other["status"]

# A failing simulation stops the case at that trial.
os.environ["FAKE_SIM_FAIL_TRIAL"] = "2"
OUT3 = ROOT / "out_sim_fail"
only_n1 = plan_file("n1.toml", HEAD + CRITERIA
                    + '[[case]]\nname = "N1"\ncriteria = "basic"\n')
status, out3, _ = run_main(only_n1, "--out", OUT3, "--bin-dir", BIN)
del os.environ["FAKE_SIM_FAIL_TRIAL"]
assert status == 1
failed = json.loads((OUT3 / "summary.json").read_text())["cases"][0]
assert failed["stop"]["reason"] == "sim_2 failed: exit 1"
assert "x7_follow_sim: simulated failure" in failed["stop"]["stderr"]
assert len(failed["trials"]) == 3 and "simulation" not in failed["trials"][2]
assert [c["verdict"] for c in failed["criteria"]] == ["FAIL"] * 5

# An existing output directory is refused; nothing is overwritten.
status, _, err = run_main(only_n1, "--out", OUT3, "--bin-dir", BIN)
assert status == 2 and "exists" in err
status, _, err = run_main(only_n1, "--out", ROOT / "x", "--jobs", "0")
assert status == 2 and "--jobs" in err
# A reference that cannot be prepared: exit 2, the summary says why.
status, _, err = run_main(only_n1, "--out", ROOT / "no_bin", "--bin-dir",
                          ROOT / "absent_bin")
assert status == 2 and "x7_plan_ptp" in err
assert "error" in json.loads((ROOT / "no_bin" / "summary.json").read_text())

# Help states the schema and the exit status.
status, out, _ = run_main("--help")
assert status == 0 and "Exit status" in out and "transfer_to" in out

# ------------------------------------------------ the shipped plans

# The acceptance criteria were fixed before the runs: they are pinned here
# as the study's brief states them, so that no edit of the plan changes
# them unnoticed.
NOMINAL = [("a", "runs_ok", {}),
           ("b", "decreasing", {"from": 0, "to": 5}),
           ("c", "ratio", {"trial": 5, "of": 0, "at_most": 0.15}),
           ("d", "no_clamp", {}),
           ("e", "hf_ratio", {"trial": 5, "of": 0, "at_most": 1.5})]
ROBUST = [NOMINAL[0], NOMINAL[1],
          ("c", "ratio", {"trial": 5, "of": 0, "at_most": 0.30}), NOMINAL[4]]
EXPECTED = {
    "N1": NOMINAL, **{f"R{i}": ROBUST for i in range(1, 15)}, "N1x8": [],
    "D1": [NOMINAL[0],
           ("b", "vs_case", {"case": "N1x8", "trials": [5, 6, 7],
                             "at_most": 1.5}),
           ("c", "bounded", {"trials": [3, 4, 5, 6, 7], "of": 0,
                             "at_most": 1.0})],
    "S1": [],
    "F1": [NOMINAL[0], ("b", "ratio", {"trial": 1, "of": 0, "at_most": 0.2}),
           ("c", "bounded", {"trials": [2, 3, 4, 5], "of": 1,
                             "at_most": 1.2})],
    "F2": [NOMINAL[0], NOMINAL[1],
           ("c", "ratio", {"trial": 5, "of": 0, "at_most": 0.1})],
    "F3": [NOMINAL[0], NOMINAL[1]],
    "F4": [NOMINAL[0], NOMINAL[1], ROBUST[2]],
    "T1": [("a", "transfer", {"at_most": 0.5})],
    "T2": [("a", "transfer", {"at_most": 0.5})],
}
acceptance = fls.load_plan("config/learn_study/acceptance.toml")
cases = {c["name"]: c for c in acceptance["cases"]}
assert list(cases) == list(EXPECTED), list(cases)
for name, criteria in EXPECTED.items():
    got = [(c["id"], c["check"], {k: v for k, v in c.items()
                                  if k not in ("id", "check")})
           for c in cases[name]["criteria"]]
    assert got == criteria, (name, got)
NOMINAL_ILC = {"phi": 0.5, "psi_s": 0.0, "advance_s": 0.05,
               "cutoff_hz": 2.0, "max_trial_step_rad": 0.05}
FRICTION = {"coulomb_friction_nm": [0.15, 0.4] + [0.15] * 6,
            "viscous_friction_nm_s": [0.05, 0.3] + [0.05] * 6}
servo_like = str(pathlib.Path("config/follow_sim_servo_like.toml").resolve())
example = str(pathlib.Path("config/follow_learn_example.toml").resolve())
for name, case in cases.items():
    assert case["follow"] == servo_like and case["learn"] == example, name
    assert case["perturb"] == [] or name == "D1", name
    assert case["reference"] == {"R12": "pick_moving", "R13": "short",
                                 "R14": "b", "T2": "b"}.get(name, "pick")
    assert case["trials"] == {"N1x8": 8, "D1": 8, "S1": 10}.get(name, 6)
    assert case["method"] == ("fel" if name[0] in "FT" else "ilc"), name
    if case["method"] == "ilc":
        expected = dict(NOMINAL_ILC, **{
            "R10": {"phi": 1.0}, "R11": {"cutoff_hz": 3.0},
            "S1": {"phi": 1.9, "advance_s": 0.0, "cutoff_hz": 20.0}}.get(
                name, {}))
        assert case["learn_set"] == {"ilc": expected}, name
    expected_set = {"R3": {"command_delay_cycles": 0},
                    "R4": {"command_delay_cycles": 2},
                    "R5": {"command_delay_cycles": 3},
                    "R8": FRICTION, "R9": {"quantize_encoder": False},
                    "F4": FRICTION}.get(name)
    assert case["follow_set"] == ({"simulation": expected_set}
                                  if expected_set else {}), name
    expected_scale = {"R1": {"position_kp": 0.5, "position_kd": 0.5},
                      "R2": {"position_kp": 2.0, "position_kd": 2.0},
                      "R6": {"position_kd": 0.5},
                      "R7": {"position_kd": 2.0}}.get(name)
    assert case["follow_scale"] == ({"simulation": expected_scale}
                                    if expected_scale else {}), name
for name, step, gain in (("F1", "least-squares", 1.0),
                         ("F2", "least-squares", 0.5),
                         ("F3", "normalised-gradient", 0.2),
                         ("F4", "least-squares", 0.5),
                         ("T1", "least-squares", 0.5),
                         ("T2", "least-squares", 0.5)):
    assert cases[name]["learn_set"] == {"fel": {"step": step,
                                                "gain": gain}}, name
assert cases["T1"]["transfer_to"] == "b"
assert cases["T2"]["transfer_to"] == "pick"
assert cases["D1"]["perturb"] == [{"trials": [2], "follow_set": {
    "simulation": {"link_force": str(pathlib.Path(
        "config/learn_study/d1_link_force.toml").resolve())}},
    "follow_scale": {}}]
assert acceptance["references"]["pick_moving"] == {
    "kind": "from", "from": "pick", "cut_leading_s": 0.6}
margins = fls.load_plan("config/learn_study/margins.toml")
assert [c["name"] for c in margins["cases"]] == (
    [f"phi={v}" for v in (0.25, 0.5, 1.0, 1.5, 1.9)]
    + [f"cutoff={v}" for v in (1.0, 2.0, 3.0, 5.0, 8.0)]
    + [f"advance={v}" for v in (0.0, 0.03, 0.05, 0.08, 0.12, 0.2)]
    + [f"psi={v}" for v in (0.0, 0.03, 0.06, 0.1)]
    + [f"cross[cutoff_hz={c},command_delay_cycles={d}]"
       for c in (2.0, 5.0) for d in (1, 3)])
for case in margins["cases"]:
    assert case["trials"] == 8 and case["criteria"] == []
    assert case["learn_set"]["ilc"] == dict(NOMINAL_ILC, **{
        key.split(".")[-1]: value for key, value in case["vary"].items()
        if key.startswith("learn.")}), case["name"]

SCRATCH.cleanup()
print("follow learn study regressions OK")
