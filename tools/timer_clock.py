"""The TSC to CLOCK_MONOTONIC conversion of the timer analysis, with a bound on
every converted value and a certificate of the clock model it rests on
(tools/timer_service.py; docs/hardware/control-rate.md, the frozen design of
revision 53 of the clock-alignment work).

Three separate results. status PASS or FAIL: the slot accounting, the sign
checks, the coverage and the model-consistency check. U: a bound on the error
of every converted value, computed from the final retained set. cert CERTIFIED
or UNSUPPORTED: whether the acquisition establishes the slope premise U uses.

rule() is the calculation, given a clock model. analyze() is the only path
that can call a U validated: it takes the model from the certificate before
calculating, refuses a smaller supplied model, checks that everything the
calculation depended on lies inside the certificate's interval, and accepts
only if the analysis and the certificate both hold; a certified U whose
analysis failed is labelled a candidate, the conversion not accepted (a
label only: nothing in the calculation changes). Missing, unreadable or
malformed evidence is UNSUPPORTED; it is never read as an empty record. The
certificate holds only for the audited kernel and the audited trace-cmd build
with its audited runtime (trace libraries and plugin set, by file hash),
identified for every use of trace-cmd it rests on (the acquisition, the
recorder's extraction and report, and the main trace's report that the
analysis read, a later replay's included).
"""

from __future__ import annotations

import bisect
import heapq
import re
import statistics as st

U_LIMIT = 5000.0        # the plan's limit on U, ns
A_INTERP = 100.0        # the largest chord term for a sampler interval to be usable, ns (a chosen allocation)
W_NS = 200_000_000      # window of the slope statistic, a diagnostic only
RES = 1.0               # timestamp resolution, ns
CAP = 0.001             # the share of verified pairs that may be left out of the conversion (candidate ceiling)
SHIFT_PLL = 2           # kernel/time: include/linux/timex.h


# --- Strict parsing -----------------------------------------------------------------------------------------------

INST_LINE = re.compile(r"\[(\d{3})\]\s+\S+\s+(\d+):\s+(\w+):\s*(.*)$")
MSR_BODY = re.compile(r"^6e0, value ([0-9a-f]+)$")
VEC_BODY = re.compile(r"^vector=236$")
KIND = {"local_timer_entry": "E", "local_timer_exit": "X", "write_msr": "W"}


class Ledger:
    """The real-time CPU's deadline writes, local timer entries and exits, in
    the report's order. Every row is taken or reported; nothing is skipped."""

    def __init__(self, back_ns: int = 0) -> None:
        self.events: list[tuple] = []
        self.problems: list[str] = []
        self.backward_steps = 0
        self.back_ns = back_ns

    def feed(self, t: int, name: str, body: str) -> None:
        where = f"{name} at {t}"
        k, v = KIND[name], None
        if k == "W":
            b = MSR_BODY.match(body)
            if not b:
                self.problems.append(f"{where}: malformed deadline write ({body})")
                return
            v = int(b.group(1), 16)
        elif not VEC_BODY.match(body):
            self.problems.append(f"{where}: unexpected vector ({body})")
            return
        if self.events:
            pt, pk, pv = self.events[-1]
            if (pt, pk, pv) == (t, k, v):
                self.problems.append(f"{where}: duplicate record")
                return
            if t < pt:
                if pt - t > self.back_ns:
                    self.problems.append(f"{where}: out of order by {pt - t} ns")
                    return
                self.backward_steps += 1
        self.events.append((t, k, v))


def parse_instance(text, rt_cpu=3):
    """The x86-tsc instance's anchors. Comment rows are headers; every other row is parsed or reported."""
    tsc, problems = [], []
    for n, ln in enumerate(text.splitlines(), 1):
        if ln.startswith("#"):
            e = re.search(r"entries-in-buffer/entries-written:\s*(\d+)/(\d+)", ln)
            if e and int(e[2]) > int(e[1]):
                problems.append(f"the instance lost {int(e[2]) - int(e[1])} entries")
            continue
        if not ln.strip():
            continue
        m = INST_LINE.search(ln)
        if not m:
            problems.append(f"line {n}: malformed row")
            continue
        cpu, ts, name, body = m.groups()
        if int(cpu) != rt_cpu or name != "local_timer_entry" or not VEC_BODY.match(body):
            problems.append(f"line {n}: unexpected record ({name}, CPU {int(cpu)})")
            continue
        t = int(ts)
        if tsc and t == tsc[-1]:
            problems.append(f"line {n}: duplicate record")
            continue
        if tsc and t < tsc[-1]:
            problems.append(f"line {n}: out of order")
            continue
        tsc.append(t)
    return tsc, problems


def parse_pairs(text):
    """A tsc_mono_pairs file: (mono before, tsc, mono after, cpu). Comment rows are
    headers; every other row is parsed or reported."""
    pairs, problems = [], []
    for n, ln in enumerate(text.splitlines(), 1):
        if not ln.strip() or ln.startswith("#"):
            continue
        f = ln.split(",")
        if len(f) != 4 or not all(x.isdigit() for x in f):
            problems.append(f"sampler line {n}: malformed row")
            continue
        a, t, b, c = (int(x) for x in f)
        if b < a:
            problems.append(f"sampler line {n}: the bracket ends before it starts")
            continue
        pairs.append((a, t, b, c))
    return pairs, problems


# --- The ledger and the slots ---------------------------------------------------------------------------------------

def build_slots(ev):
    """One slot per entry. Its delimiter A is the earliest non-zero deadline armed since the previous entry and after the
    last zero write. It delimits a slot and is a conservative earlier bound; it is not asserted to be the deadline that
    caused the interrupt. A zero write disarms: deadlines written before it survive only as pending candidates, and a slot
    that has nothing else is flagged for the expiry proof in validation."""
    out, armed, pre_zero, t_zero, open_, tail_orphan, head_orphan = [], [], [], None, False, 0, 0
    for t, k, v in ev:
        if k == "W":
            if v == 0:
                pre_zero += armed
                armed, t_zero = [], t
            else:
                armed.append(v)
        elif k == "E":
            A, pending = (min(armed), False) if armed else ((min(pre_zero), True) if pre_zero else (None, False))
            out.append(dict(t=t, A=A, pending=pending, t_zero=t_zero if pending else None, nested=open_, closed=False))
            armed, pre_zero, t_zero, open_ = [], [], None, True
        else:
            if open_:
                out[-1]["closed"], open_ = True, False
            elif out:
                tail_orphan += 1
            else:
                head_orphan += 1
    return out, tail_orphan, head_orphan


def assign(sl, tsc):
    """Assignment in TSC units. No clock mapping."""
    idx = [j for j, s in enumerate(sl) if s["A"] is not None]
    A = [sl[j]["A"] for j in idx]
    notinc = sum(1 for a, b in zip(A, A[1:]) if b <= a)
    hit, before = {}, 0
    for i, t in enumerate(tsc):
        q = bisect.bisect_right(A, t) - 1
        if q < 0:
            before += 1
            continue
        hit.setdefault(idx[q], []).append(i)
    if not hit:
        return dict(first=None, why=["no instance anchor falls in any deadline slot"])
    first, last = min(hit), max(hit)
    # A partial trace boundary: the final entry may lack its exit only if it is the last entry of the trace. It leaves
    # the proof span; anything the required window needs from it then fails coverage.
    unclosed_tail = 0
    if not sl[last]["closed"] and last == len(sl) - 1 and first < last:
        del hit[last]
        last -= 1
        while last not in hit and last > first:
            last -= 1
        unclosed_tail = 1
    inner = sl[first:last + 1]
    return dict(first=first, last=last, before=before, notinc=notinc, unclosed_tail=unclosed_tail,
                pairs={j: v[0] for j, v in hit.items() if len(v) == 1},
                double=sum(1 for v in hit.values() if len(v) > 1), empty=sum(1 for j in range(first, last + 1) if j not in hit),
                nofresh=sum(1 for s in inner if s["A"] is None), nested=sum(1 for s in inner if s["nested"]),
                unclosed=sum(1 for s in inner if not s["closed"]))


# --- The clock model ------------------------------------------------------------------------------------------------

class Model:
    """ds: the slope range the bound uses (dimensionless). W_ns: the allocation for how long a reader may use the snapshot
    before an update; the two clock-read paths differ by at most ds * W there, and read the same function otherwise."""
    def __init__(self, ds, W_ns=10_000_000, label="exploratory"):
        self.ds, self.W, self.label = ds, W_ns, label
        self.R = RES + ds * W_ns
        self.g_max = 4 * A_INTERP / ds


class SamplerMap:
    """Piecewise-linear between one CPU's bracket midpoints. The windowed slope range is a diagnostic only."""
    def __init__(self, pairs, model):
        self.p = sorted((t, (a + b) / 2, (b - a) / 2) for a, t, b, _ in pairs)
        self.t, self.mono, self.m = [x[0] for x in self.p], [x[1] for x in self.p], model
        self.dep_lo, self.dep_hi = None, None  # the extent of the sampler brackets the calculation actually used

    def slope_statistic(self):
        tm, tc, out, start = self.mono, self.t, [], self.mono[0]
        while True:
            a, b = bisect.bisect_left(tm, start), bisect.bisect_left(tm, start + W_NS)
            if b >= len(tm):
                break
            if b > a:
                out.append((tm[b] - tm[a]) / (tc[b] - tc[a]))
            start += W_NS
        return (max(out) - min(out)) / st.mean(out) if len(out) > 1 else None

    def at(self, t):
        """(mono, bound on |map - main-trace clock|), or None outside the span or across an unusable interval."""
        k = bisect.bisect_right(self.t, t)
        if k == 0 or k >= len(self.t):
            return None
        (t0, m0, h0), (t1, m1, h1) = self.p[k - 1], self.p[k]
        g = m1 - m0
        if g > self.m.g_max:
            return None
        lo, hi = m0 - h0, m1 + h1
        self.dep_lo = lo if self.dep_lo is None else min(self.dep_lo, lo)
        self.dep_hi = hi if self.dep_hi is None else max(self.dep_hi, hi)
        return m0 + (m1 - m0) * (t - t0) / (t1 - t0), max(h0, h1) + self.m.ds * g / 4 + RES + self.m.R

    def gaps_over(self, lo, hi):
        return [(a, b) for a, b in zip(self.mono, self.mono[1:]) if b - a > self.m.g_max and b >= lo and a <= hi]


def interval_bound(a, b, model):
    """A bound on |converted value - main-trace clock| anywhere between two retained anchors: each endpoint's error, weighted
    at most fully, plus the chord of the clock over the interval, the clock-read difference and the resolution."""
    return max(a["E"], b["E"]) + model.ds * (b["t"] - a["t"]) / 4 + model.R + RES


def select(items, model, cap_n):
    """Which verified pairs feed the conversion, and U. Finite: each step leaves one pair out, at most cap_n steps. Returns the
    state with the smallest U seen, recomputed from that final set; there is no intermediate budget to converge."""
    n = len(items)
    prev, nxt, alive = list(range(-1, n - 1)), list(range(1, n + 1)), [True] * n
    heap = [(-interval_bound(items[i], items[i + 1], model), i, i + 1) for i in range(n - 1)]
    heapq.heapify(heap)
    removed, best_u, best_k = [], float("inf"), 0
    while heap:
        while heap and not (alive[heap[0][1]] and alive[heap[0][2]] and nxt[heap[0][1]] == heap[0][2]):
            heapq.heappop(heap)
        if not heap:
            break
        u, a, b = -heap[0][0], heap[0][1], heap[0][2]
        if u < best_u:
            best_u, best_k = u, len(removed)
        if len(removed) >= cap_n:
            break
        ia, ib = items[a], items[b]
        if model.ds * (ib["t"] - ia["t"]) / 4 >= max(abs(ia["e"]), abs(ib["e"])):
            break  # the interval's length sets the bound; leaving out an endpoint cannot lower it
        v = a if ia["E"] >= ib["E"] else b
        if prev[v] < 0 or nxt[v] >= n:
            break  # the span's ends stay; coverage is decided on them
        p, q = prev[v], nxt[v]
        alive[v], nxt[p], prev[q] = False, q, p
        removed.append(v)
        heapq.heappush(heap, (-interval_bound(items[p], items[q], model), p, q))
    out = set(removed[:best_k])
    kept = [items[i] for i in range(n) if i not in out]
    bounds = [interval_bound(a, b, model) for a, b in zip(kept, kept[1:])]
    return kept, [items[i] for i in removed[:best_k]], bounds


class Conversion:
    """The conversion, with the local bound of each interval. Same interpolation as tools/timer_service.py's TscMap."""
    def __init__(self, kept, bounds):
        self.t = [k["tsc"] for k in kept]
        self.m = [k["t"] for k in kept]
        self.b = bounds

    def at(self, x):
        k = bisect.bisect_right(self.t, x)
        if k == 0 or x > self.t[-1]:
            return None
        if k >= len(self.t):
            return float(self.m[-1]), self.b[-1]
        t0, t1, m0, m1 = self.t[k - 1], self.t[k], self.m[k - 1], self.m[k]
        return m0 + (m1 - m0) * (x - t0) / (t1 - t0), self.b[k - 1]


def rule(ev, tsc, samp_pairs, model, window=None, cap=None):
    """ev: time-ordered (t, 'W'|'E'|'X', value)."""
    why = []
    sl, tail_orphan, head_orphan = build_slots(ev)
    r = assign(sl, tsc)
    if r["first"] is None:
        return dict(status="FAIL", why=r["why"], U=None)
    for key, msg in (("nofresh", "entries with no deadline written since the previous entry"), ("nested", "entries inside the previous interrupt"),
                     ("notinc", "times the deadline ladder decreases"), ("double", "deadline slots hold two instance anchors"),
                     ("empty", "deadline slots hold no instance anchor"), ("unclosed", "entries inside the span have no exit")):
        if r[key]:
            why.append(f"{r[key]} {msg}")
    if tail_orphan:
        why.append(f"{tail_orphan} exits with no entry")
    M = SamplerMap(samp_pairs, model)
    items, offs, bad_e, bad_dl, nomap, bad_pending = [], set(), 0, 0, 0, 0
    for j, i in sorted(r["pairs"].items()):
        offs.add(j - i)
        s = sl[j]
        v, a = M.at(tsc[i]), M.at(s["A"])
        if s["pending"] and (a is None or s["t_zero"] - a[0] <= a[1]):
            bad_pending += 1
        if v is None or a is None:
            nomap += 1
            continue
        e = s["t"] - v[0]
        if e > v[1]:
            bad_e += 1
        if s["t"] - a[0] < -a[1]:
            bad_dl += 1
        items.append(dict(tsc=tsc[i], t=s["t"], e=e, B=v[1], E=abs(e) + v[1], j=j, i=i))
    if len(offs) != 1:
        why.append(f"the pairs do not share one index offset ({len(offs)} offsets)")
    if bad_e:
        why.append(f"{bad_e} pairs have the main stamp after the mapped instance stamp, beyond the bound")
    if bad_dl:
        why.append(f"{bad_dl} interrupts are stamped before their deadline, beyond the bound")
    if bad_pending:
        why.append(f"{bad_pending} interrupts follow a deadline disarmed before it is known to have expired")
    # Coverage comes first and does not depend on which pairs are retained.
    if len(items) < 2:
        why.append(f"no usable conversion pairs ({len(items)} verified, {nomap} without an independent check)")
        return dict(status="FAIL", why=why, U=None, paired=len(r["pairs"]), used=0, nomap=nomap, ladder=r, slots=sl)
    items.sort(key=lambda x: x["tsc"])
    cap_n = int((CAP if cap is None else cap) * len(r["pairs"]))
    kept, out, bounds = select(items, model, cap_n)
    U = max(bounds)
    conv = Conversion(kept, bounds)
    if window:
        lo, hi = window
        un = M.gaps_over(lo, hi)
        if un:
            why.append(f"{len(un)} sampler intervals longer than {model.g_max / 1e6:.1f} ms overlap the run (longest {max(b - a for a, b in un) / 1e6:.1f} ms)")
        if not (kept[0]["t"] <= lo and kept[-1]["t"] >= hi):
            why.append("the pairs do not cover the run window")
    if U > U_LIMIT:
        why.append(f"U {U:.0f} ns exceeds {U_LIMIT:.0f} ns with {len(out)} of at most {cap_n} pairs left out")
    # A falsification check of the model at every sampler pair inside the span; not a bound.
    viol = worst = 0
    for a, t, b, _ in samp_pairs:
        c = conv.at(t)
        if c is None:
            continue
        h = (b - a) / 2
        ratio = abs(c[0] - (a + b) / 2) - (c[1] + h + model.R + RES)
        worst = max(worst, ratio)
        viol += ratio > 0
    if viol:
        why.append(f"{viol} sampler pairs lie outside the bound: the clock model does not describe this capture")
    gaps = [b["t"] - a["t"] for a, b in zip(kept, kept[1:])]
    return dict(status="FAIL" if why else "PASS", why=why, U=U, U_label=model.label, dependency=dict(sampler=(M.dep_lo, M.dep_hi), span=(kept[0]["t"], kept[-1]["t"])),
                paired=len(r["pairs"]), used=len(kept), left_out=[round(x["e"]) for x in out], nomap=nomap, Bmax=max(k["B"] for k in kept), Emax=max(k["E"] for k in kept), emin=min(k["e"] for k in kept), emax=max(k["e"] for k in kept),
                g_conv_ms=max(gaps) / 1e6, Imax=model.ds * max(gaps) / 4, offsets=sorted(offs)[:3], pairs=[(k["tsc"], k["t"]) for k in kept], conv=conv, ladder=r, unclosed_tail=r["unclosed_tail"], head_orphan=head_orphan, slope_statistic=M.slope_statistic(), residual_margin=worst)


# --- Certification: whether the acquisition establishes the model the calculation uses ---------------------------------

SCHEMA = 1
AUDITED = dict(release="7.0.0-42-generic", package="7.0.0-42.42", config_sha256="5c1ce5c46247b47a133cf80e7f852fd9d877c832c89848ab42b4db3092fae7aa")
# The audited trace-cmd (source trace-cmd_3.3.3.orig.tar.gz, the design's source audit): the installed package's executable,
# whose SHA-256 is that of the file dpkg's own checksum record verifies. Another build or version needs a new audit.
AUDITED_TRACE_CMD = dict(package="trace-cmd", version="3.3.3-1ubuntu2", architecture="amd64",
                         sha256="dbcc6cafe1686b7af192868ac9daac719bafa8bdf225b66459c025dca44b0882")
TRACE_CMD_USES = dict(before="that started the acquisition", after="that extracted and reported the recorder",
                      report="that reported the main trace for this analysis")
# The runtime the audited trace-cmd runs with, under the launch (tools/tk_record.py: the loader's and libtraceevent's
# search inputs removed): the trace libraries the dynamic loader resolves, and the plugin set libtraceevent loads, each
# file by SHA-256 (the reviewer's decision of 2026-11-11; the source packages' files match the signed index). Another
# library, plugin or search input needs a review before its acquisitions can be certified.
_TE_PLUGINS = "/usr/lib/x86_64-linux-gnu/traceevent/plugins"
_TE_PLUGIN_SHA256 = dict(
    plugin_cfg80211="c96ccb32b3d66149da9780a05e7d35a4ad5dfcd6774a9e393c0b00e623272ae9",
    plugin_function="d8f516bd5725499a14ecee3a246cb0f1153ebf3088e6a0bb552330b8b1bde88e",
    plugin_futex="2afc9baf4e16df1cf52ed352fa5bd5608b968fea09769d4d9b3e7c24547adb7b",
    plugin_hrtimer="7857fe346bc743a445bdb7b113d6ec7c0d17c8370d00ca0471dab507fa11408f",
    plugin_jbd2="e07dc2e9d4245c4a56835dfd0430f6a4d028ecc9ac6746c5df7fd466ecf5a989",
    plugin_kmem="339f76c483cc12aea44dd4ea77f748e7815adf23b02dc3d1394f8498ff4c6f13",
    plugin_kvm="835bd1d9e9f57eef99d1fcb8923ec52a731b000bebaca6c5265a552c2dabd13c",
    plugin_mac80211="93e6cf27caf51bd57142846a477606514faf31aadd44545335f2127448efb51e",
    plugin_net="1076dc8c778f965a31009d8de4f9e5cd61b98f836f5fb1eecb74b2bd8e1c3799",
    plugin_sched_switch="20835eef3908e171262953bbfc6c1d4ea7a16310f0f90aaca0b5945ab58df9fb",
    plugin_scsi="03971e30b4f417f9d2b50ae5a3c36aa8b226052cf61304efc7848a2da33399d6",
    plugin_tlb="e32d697de11eab1601a6d7f3dc1f6b853614910a16e2718f5beade2ce890189d",
    plugin_xen="1ad4921c1cc847278bc48c01da7a81b6164390d9f7fdb114166f90e159f07237")
AUDITED_TRACE_RUNTIME: dict = dict(
    inputs=dict(HOME=None, TRACEEVENT_PLUGIN_DIR=None, TRACECMD_PLUGIN_DIR=None, LD=[]),
    preload="none",
    libraries={
        "libtraceevent.so.1": dict(path="/usr/lib/x86_64-linux-gnu/libtraceevent.so.1.8.7", sha256="7e401c9eabc1718605fd8df406bc6c1bb9308e153b18b10cd14108aeb2621158",
                                   package="libtraceevent1", version="1:1.8.7-1", architecture="amd64"),
        "libtracefs.so.1": dict(path="/usr/lib/x86_64-linux-gnu/libtracefs.so.1.8.2", sha256="b3a61292bfe774fcce3b1bae762e1ff211ff35221a24078d2cf8375f4b7b8c95",
                                package="libtracefs1", version="1.8.2-1ubuntu1", architecture="amd64")},
    plugin_dirs=[_TE_PLUGINS],
    plugins={f"{_TE_PLUGINS}/{n}.so": dict(path=f"{_TE_PLUGINS}/{n}.so", sha256=h, package="libtraceevent1-plugin", version="1:1.8.7-1", architecture="amd64")
             for n, h in _TE_PLUGIN_SHA256.items()})


IDENTITY_FIELDS = {"read_status", "path", "sha256", "package", "version", "architecture", "runtime"}
RUNTIME_FIELDS = {"inputs", "preload", "loaded", "libraries", "plugin_dirs", "plugins"}


def _described(e):
    if not isinstance(e, dict):
        return "absent"
    return f"{e.get('package')} {e.get('version')}, {e.get('path')}, sha256 {str(e.get('sha256'))[:16]}"


def runtime_differences(rt):
    """Each way a recorded runtime differs from the audited one; none when it is the audited one."""
    if not isinstance(rt, dict):
        return ["the runtime is not a mapping"]
    a, out = AUDITED_TRACE_RUNTIME, []
    if rt.get("inputs") != a["inputs"]:
        out.append(f"search inputs {rt.get('inputs')!r}")
    if rt.get("preload") != a["preload"]:
        out.append(f"a preload: {str(rt.get('preload'))[:80]!r}")
    libs = rt.get("libraries") if isinstance(rt.get("libraries"), dict) else {}
    for soname in sorted(set(libs) | set(a["libraries"])):
        if libs.get(soname) != a["libraries"].get(soname):
            out.append(f"{soname}: {_described(libs.get(soname))}" if soname in a["libraries"] else f"an unreviewed library {soname}")
    if rt.get("plugin_dirs") != a["plugin_dirs"]:
        out.append(f"plugin directories {rt.get('plugin_dirs')!r}")
    plugins = rt.get("plugins") if isinstance(rt.get("plugins"), dict) else {}
    for p in sorted(set(plugins) | set(a["plugins"])):
        if plugins.get(p) != a["plugins"].get(p):
            out.append(f"plugin {p} not loaded" if p not in plugins else f"an unreviewed plugin {p}" if p not in a["plugins"]
                       else f"plugin {p}: {_described(plugins[p])}")
    return out
W_OP_NS = 10_000_000            # the operational maximum on an update's duration; every update in the interval must meet it
SETTLE_NS = 2_000_000           # two NTP intervals at HZ=1000: a later update is then certain to accumulate and run timekeeping_adjust
READ_ONLY_MODES = {0, 0xa001}   # adjtimex modes that change nothing: a read, and ADJ_OFFSET_SS_READ
PROBES = ("ntp_adjtimex", "ntp_clear", "update_entry", "update_return")
STA_NANO = 0x2000
STA_REFUSED = 0x0f36            # PPSFREQ, PPSTIME, leap insert/delete, PPSSIGNAL, PPSJITTER, PPSWANDER, PPSERROR
SHIFT_MAX = SHIFT_PLL + 10
OFFSET_TOLERANCE_NS = 3         # adjtimex floors the offset to 1 ns at each of the two reads, plus 1 ns of accumulated rounding
MULT_MARGIN = 1.001             # covers mult differing from its registered value by at most MAX_DEVIATION_PPM: 1 / (1 - 900e-6) < 1.001
MAX_DEVIATION_PPM = 900         # the adjtimex frequency plus the offset term; the certificate refuses anything larger
NOMINAL_TICK_US = 10_000        # USER_TICK_USEC at USER_HZ=100, the baseline ntp_update_frequency scales from
MAX_FREQ = 32_768_000           # MAXFREQ, 500 ppm, in adjtimex units of 2^-16 ppm


def calc_mult_shift(from_khz, to=1_000_000, maxsec=600_000):
    """kernel/time/clocksource.c clocks_calc_mult_shift, for a clocksource registered in kHz."""
    tmp, sftacc = (maxsec * from_khz) >> 32, 32
    while tmp:
        tmp >>= 1
        sftacc -= 1
    sft = 32
    for sft in range(32, 0, -1):
        tmp = ((to << sft) + from_khz // 2) // from_khz
        if (tmp >> sftacc) == 0:
            break
    return tmp, sft


class _Schema:
    """Typed access to the evidence. A field that is absent, of the wrong type or out of range is a recorded reason."""
    def __init__(self, ev):
        self.ev, self.why = ev, []

    def get(self, path, typ, lo=None, hi=None):
        node = self.ev
        for key in path.split("."):
            if not isinstance(node, dict) or key not in node:
                self.why.append(f"missing {path}")
                return None
            node = node[key]
        if isinstance(node, bool) != (typ is bool) or not isinstance(node, typ):
            self.why.append(f"{path} has the wrong type")
            return None
        if (lo is not None and node < lo) or (hi is not None and node > hi):
            self.why.append(f"{path} is out of range")
            return None
        return node


# lib/ctype.c: the spaces of the kernel's isspace().
KERNEL_SPACE = frozenset("\t\n\v\f\r \xa0")


def boot_params(cmdline):
    """The (name, value) pairs of a command line as the kernel splits them: lib/cmdline.c
    next_arg, which has its own quoting rules (a general shell tokenizer differs). The whole
    line is scanned, including anything after "--"."""
    out, pos, n = [], 0, len(cmdline)
    while pos < n and cmdline[pos] in KERNEL_SPACE:
        pos += 1
    while pos < n:
        args = cmdline[pos:]
        in_quote = quoted = False
        if args[0] == '"':
            args = args[1:]
            pos += 1
            in_quote = quoted = True
        i = equals = 0
        while i < len(args):
            c = args[i]
            if c in KERNEL_SPACE and not in_quote:
                break
            if equals == 0 and c == "=":
                equals = i
            if c == '"':
                in_quote = not in_quote
            i += 1
        end = i
        if quoted and i > 0 and args[i - 1] == '"':
            end = i - 1
        if equals:
            start = equals + 1
            if args[start:start + 1] == '"':
                start += 1
                if args[i - 1] == '"':
                    end = i - 1
            out.append((args[:equals], args[start:max(start, end)]))
        else:
            out.append((args[:end], None))
        pos += i + (1 if i < len(args) else 0)
        while pos < n and cmdline[pos] in KERNEL_SPACE:
            pos += 1
    return out


def sets_ntp_tick_adj(cmdline):
    """Whether a command line names ntp_tick_adj: __setup matches dashes as underscores."""
    return any(name.replace("-", "_") == "ntp_tick_adj" for name, _ in boot_params(cmdline))


def _unsupported(reasons):
    return dict(status="UNSUPPORTED", reasons=reasons)


def certify(ev):
    """The certificate for one acquisition. Its interval D runs from the state read before the capture to the state read after it.
    Returns the slope range and the clock-read maximum that the calculation must use, with their provenance."""
    if ev is None:
        return _unsupported(["no timekeeping record for this capture"])
    if not isinstance(ev, dict):
        return _unsupported(["the timekeeping record is not a mapping"])
    S = _Schema(ev)
    # Stage 1: everything required is present, typed and in range.
    T = (0, 2**62)
    schema = S.get("schema", int)
    for sec in ("kernel", "clocksource", "log", "recorder", "recorder.buffer", "state.before", "state.after"):
        st_ = S.get(f"{sec}.read_status", str)
        if st_ is not None and st_ != "ok":
            S.why.append(f"{sec} was not read successfully ({st_})")
    release, config, khz = S.get("kernel.release", str), S.get("kernel.config_sha256", str), S.get("kernel.tsc_khz", int, 100_000, 20_000_000)
    cmdline = S.get("kernel.cmdline", str)
    cs_b, cs_a = S.get("clocksource.before", str), S.get("clocksource.after", str)
    covers, events = S.get("log.covers", list), S.get("log.clock_events", list)
    armed, confirmed = S.get("recorder.armed_t", int, *T), S.get("recorder.confirmed_t", int, *T)
    calls, updates = S.get("recorder.calls", list), S.get("recorder.updates", list)
    un_e, un_r = S.get("recorder.unmatched.entries", list), S.get("recorder.unmatched.returns", list)
    for k in ("overrun", "dropped"):
        n = S.get(f"recorder.buffer.{k}", int, 0)
        if n:
            S.why.append(f"the recorder's buffer reports {n} {k}")
    probes = {}
    for name in PROBES:
        probes[name] = {k: S.get(f"recorder.probes.{name}.{k}", typ, *rng) for k, typ, rng in (("enabled_before", bool, ()), ("enabled_after", bool, ()), ("hits", int, (0,)), ("missed", int, (0,)))}
    tools = {}
    for use, desc in TRACE_CMD_USES.items():
        st_ = S.get(f"trace_cmd.{use}.read_status", str)
        if st_ is not None and st_ != "ok":
            S.why.append(f"the trace-cmd {desc} was not identified ({st_})")
        elif st_ is not None:
            tools[use] = {k: S.get(f"trace_cmd.{use}.{k}", str) for k in AUDITED_TRACE_CMD}
            tools[use]["runtime"] = S.get(f"trace_cmd.{use}.runtime", dict)
            # Read whole, as the other evidence: a field the certificate does not know is not passed over.
            extra = sorted(set(ev["trace_cmd"][use]) - IDENTITY_FIELDS)
            if extra:
                S.why.append(f"the trace-cmd {desc} has an identity field the certificate does not know: {extra[0]}")
            if isinstance(tools[use]["runtime"], dict) and set(tools[use]["runtime"]) != RUNTIME_FIELDS:
                S.why.append(f"the trace-cmd {desc} has a runtime with other fields than {sorted(RUNTIME_FIELDS)}")
    st = {}
    for side in ("before", "after"):
        st[side] = {k: S.get(f"state.{side}.{k}", int, *rng) for k, rng in (("t_start", T), ("t_end", T), ("tid", (1,)), ("ret", (0, 5)), ("offset", (-(2**40), 2**40)), ("freq", (-MAX_FREQ, MAX_FREQ)), ("status", (0, 0xffff)), ("constant", (0, 10)), ("tick", (1, 2**31)), ("time_adjust", (-(2**40), 2**40)))}
    if covers is not None and not (len(covers) == 2 and all(isinstance(x, int) and not isinstance(x, bool) for x in covers) and covers[0] < covers[1]):
        S.why.append("log.covers is not a time range")
        covers = None
    if events is not None and not all(isinstance(x, str) for x in events):
        S.why.append("log.clock_events holds something other than messages")
    for lst, name in ((un_e, "entries"), (un_r, "returns")):
        if lst is not None and not all(isinstance(x, int) and not isinstance(x, bool) for x in lst):
            S.why.append(f"recorder.unmatched.{name} holds something other than times")
    if calls is not None:
        for c in calls:
            if not (isinstance(c, dict) and c.get("name") in ("ntp_adjtimex", "ntp_clear") and all(isinstance(c.get(k), int) and not isinstance(c.get(k), bool) for k in ("t", "tkid", "tid", "modes")) and 0 <= c["tkid"] < 16 and c["modes"] >= 0 and c["t"] >= 0):
                S.why.append("recorder.calls holds a malformed record")
                break
    if updates is not None:
        for u in updates:
            if not (isinstance(u, dict) and all(isinstance(u.get(k), int) and not isinstance(u.get(k), bool) for k in ("entry", "ret", "cpu")) and u["ret"] >= u["entry"] >= 0 and u["cpu"] >= 0):
                S.why.append("recorder.updates holds a malformed record")
                break
    if S.why:
        return _unsupported(S.why)
    # Stage 2: the conditions.
    why = S.why
    a, b = st["before"], st["after"]
    if schema != SCHEMA:
        why.append(f"evidence schema {schema}, expected {SCHEMA}")
    if release != AUDITED["release"] or config != AUDITED["config_sha256"]:
        why.append("the kernel or its configuration is not the audited one")
    for use, t in tools.items():
        if {k: t[k] for k in AUDITED_TRACE_CMD} != AUDITED_TRACE_CMD:
            why.append(f"the trace-cmd {TRACE_CMD_USES[use]} is not the audited build ({AUDITED_TRACE_CMD['package']} {AUDITED_TRACE_CMD['version']}): "
                       f"{t['package']} {t['version']} {t['architecture']}, sha256 {t['sha256'][:16]}")
        diffs = runtime_differences(t["runtime"])
        if diffs:
            why.append(f"the trace-cmd {TRACE_CMD_USES[use]} ran with an unaudited runtime ({len(diffs)} differences): " + "; ".join(diffs[:3]))
    # The baseline the multiplier unit rests on: the nominal tick, and no boot-time calibration. __setup matches dashes as underscores.
    if sets_ntp_tick_adj(cmdline):
        why.append("the boot command line sets ntp_tick_adj")
    if cs_b != "tsc" or cs_a != "tsc":
        why.append("the clocksource is not tsc before and after")
    if not (a["t_start"] <= a["t_end"] < b["t_start"] <= b["t_end"]):
        why.append("the state reads are not in order")
    D = (a["t_start"], b["t_end"])
    if covers[0] > D[0] or covers[1] < D[1]:
        why.append("the kernel log does not cover the interval")
    elif events:
        why.append(f"kernel log: {events[0]}")
    if not armed < a["t_start"]:
        why.append("the recorder was not armed before the first state read")
    if not confirmed > b["t_end"]:
        why.append("the recorder was not confirmed after the last state read")
    for name, p in probes.items():
        if not (p["enabled_before"] and p["enabled_after"]):
            why.append(f"probe {name} is not confirmed enabled")
        if p["missed"]:
            why.append(f"probe {name} missed {p['missed']} events")
    n_recorded = dict(ntp_adjtimex=sum(c["name"] == "ntp_adjtimex" for c in calls), ntp_clear=sum(c["name"] == "ntp_clear" for c in calls), update_entry=len(updates) + len(un_e), update_return=len(updates) + len(un_r))
    for name, n in n_recorded.items():
        if probes[name]["hits"] != n:
            why.append(f"probe {name} hit {probes[name]['hits']} times but {n} records are present")
    # The state reads are themselves recorded by the locked-path probe, so the interval's ends are serialised against every change.
    for side, s in st.items():
        if not any(c["name"] == "ntp_adjtimex" and c["tkid"] == 0 and c["modes"] == 0 and c["tid"] == s["tid"] and s["t_start"] <= c["t"] <= s["t_end"] for c in calls):
            why.append(f"the {side} state read is not in the record")
    change = [c for c in calls if c["tkid"] == 0 and (c["name"] == "ntp_clear" or c["modes"] not in READ_ONLY_MODES) and armed <= c["t"] <= b["t_end"]]
    if change:
        why.append(f"{len(change)} timekeeping changes between arming and the last state read (first: {change[0]['name']} at {change[0]['t']})")
    # An update that begins after arming and accumulates has absorbed any change made before arming.
    after_arm = sorted(u["entry"] for u in updates if u["entry"] > armed)
    settled = None
    if after_arm:
        first = after_arm[0]
        rets = [u["ret"] for u in updates if u["entry"] >= first + SETTLE_NS]
        settled = min(rets) if rets else None
    if settled is None or settled >= a["t_start"]:
        why.append("no accumulating update completed between arming and the first state read")
    # Unmatched records are not symmetric. An entry has no confirmed end, so it may overlap the interval whenever it began; only one
    # after the interval's end cannot. A return shows that an earlier invocation ended, so one before the first complete update is harmless.
    if any(t <= D[1] for t in un_e):
        why.append("an unmatched update entry before the end of the interval has no recorded return")
    first_complete = after_arm[0] if after_arm else None
    if any(t <= D[1] and (first_complete is None or t >= first_complete) for t in un_r):
        why.append("an unmatched update return inside the interval")
    # The clock-read premise: every update that overlaps the interval finished within the operational maximum.
    inside = [u for u in updates if u["ret"] >= D[0] and u["entry"] <= D[1]]
    max_age = max((u["ret"] - u["entry"] for u in inside), default=None)
    if max_age is None:
        why.append("no update is recorded inside the interval")
    elif max_age > W_OP_NS:
        why.append(f"an update took {max_age} ns, over the {W_OP_NS} ns maximum")
    for k in ("freq", "constant", "tick"):
        if a[k] != b[k]:
            why.append(f"adjtimex {k} changed")
    for name, s in st.items():
        if s["tick"] != NOMINAL_TICK_US:
            why.append(f"adjtimex tick is {s['tick']} us, not the audited {NOMINAL_TICK_US} us")
        if s["time_adjust"] != 0:
            why.append("an adjtime slew is pending")
        if s["status"] & STA_REFUSED or not s["status"] & STA_NANO:
            why.append("the NTP status is outside the audited path")
    if why:
        return _unsupported(why)
    # The recurrence, and the slope range it implies.
    sh = SHIFT_PLL + a["constant"]
    span_lo, span_hi = (b["t_start"] - a["t_end"]) / 1e9, (b["t_end"] - a["t_start"]) / 1e9
    candidates = range(int(span_lo), int(span_hi) + 2)
    matched = False
    for n in candidates:
        o = float(a["offset"])
        for _ in range(n):
            o -= o / 2 ** sh
        matched = matched or abs(o - b["offset"]) <= OFFSET_TOLERANCE_NS
    if not matched:
        return _unsupported([f"the adjtimex offset went from {a['offset']} to {b['offset']}, which the recurrence does not produce"])
    deviation_ppm = abs(a["freq"]) / 65536 + max(abs(a["offset"]), abs(b["offset"])) / (2 ** sh - 1) / 1e3
    if deviation_ppm > MAX_DEVIATION_PPM:
        return _unsupported([f"the frequency deviation of {deviation_ppm:.0f} ppm is more than the multiplier margin covers ({MAX_DEVIATION_PPM} ppm)"])
    mult, shift = calc_mult_shift(khz)
    unit = MULT_MARGIN / mult
    chunk = (abs(a["offset"] - b["offset"]) / (2 ** sh - 1) + 2 / 2 ** sh) / 1e9
    ds = 2 * unit + chunk
    return dict(status="CERTIFIED", reasons=[], ds=ds, W_ns=W_OP_NS, D=D,
                provenance=dict(audit=AUDITED["package"], trace_cmd=f"{AUDITED_TRACE_CMD['package']} {AUDITED_TRACE_CMD['version']}",
                                trace_runtime=", ".join(f"{e['package']} {e['version']}" for e in AUDITED_TRACE_RUNTIME["libraries"].values())
                                + f", {len(AUDITED_TRACE_RUNTIME['plugins'])} plugins", mult=mult, shift=shift, unit_ppm=unit * 1e6, chunk_ppm=chunk * 1e6, observed_max_update_ns=max_age, updates_in_interval=len(inside), tick_us=a["tick"], ntp_tick_adj="not on the boot command line", deviation_ppm=deviation_ppm))


def analyze(main_parse, inst_parse, samp_pairs, window=None, evidence=None, model=None, cap=None):
    """main_parse: (events, problems) from a Ledger; inst_parse: (tsc, problems) from parse_instance.
    With a certificate the model comes from it; a supplied model is honoured only if it is at least as large. Without one, a supplied
    model is exploratory. accepted requires the analysis and the certificate to both hold."""
    cert = certify(evidence)
    reasons = list(cert["reasons"])
    certified = cert["status"] == "CERTIFIED"
    if certified and model is None:
        use, source = Model(cert["ds"], cert["W_ns"], label="certified"), "certificate"
    elif certified and model.ds >= cert["ds"] and model.W >= cert["W_ns"]:
        use, source = Model(model.ds, model.W, label="certified"), "supplied, not smaller than the certificate"
    elif certified:
        use, source, certified = Model(model.ds, model.W, label="exploratory"), "supplied, smaller than the certificate", False
        reasons.append(f"the calculation used ds {model.ds:.3g} and W {model.W:.3g}, smaller than the certificate's {cert['ds']:.3g} and {cert['W_ns']:.3g}")
    elif model is not None:
        use, source = Model(model.ds, model.W, label="exploratory"), "exploratory, no certificate"
    else:
        return dict(status="NOT ANALYSED", why=["no certificate and no exploratory model"], accepted=False, U=None, U_label=None, cert="UNSUPPORTED", cert_reasons=reasons, model=None)
    problems = list(main_parse[1]) + list(inst_parse[1])
    r = rule(main_parse[0], inst_parse[0], samp_pairs, use, window=window, cap=cap)
    r["why"] = problems + r["why"]
    if problems:
        r["status"] = "FAIL"
    if certified and "dependency" in r:
        lo, hi = cert["D"][0], cert["D"][1]
        dep = r["dependency"]
        for name, (x, y) in (("sampler brackets", dep["sampler"]), ("conversion span", dep["span"])):
            if x is None or x <= lo or y >= hi:
                certified = False
                reasons.append(f"the {name} used by the calculation are not inside the certificate's interval")
    elif certified:
        certified = False
        reasons.append("the calculation produced no dependency extent")
    # The label only presents the result (the accounting review of 2026-11-11 approved it as a reporting change): U is
    # validated only when the certificate and the rule both hold. A certified U whose rule failed (the deadline-slot
    # accounting, say) is a candidate, printed with the conversion not accepted; the certificate stays visible apart.
    accepted = bool(certified and r["status"] == "PASS")
    label = "validated" if accepted and r.get("U") is not None else "candidate U; conversion not accepted" if certified else "exploratory"
    r.update(cert="CERTIFIED" if certified else "UNSUPPORTED", cert_reasons=reasons, U_label=label,
             model=dict(ds=use.ds, W_ns=use.W, R_ns=use.R, source=source), provenance=cert.get("provenance"), accepted=accepted)
    return r
