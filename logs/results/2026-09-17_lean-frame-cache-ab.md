# Lean launch with and without the per-frame segmentation cache — dinosaur, 2026-09-17

**Question:** on the *lean* stack (not the competition preset), what does caching
the per-frame road segmentation actually buy?

**Answer:** it skips exactly the frames it claims to — **20.0-20.2% of
segmentations, every run** — and costs nothing. The CPU saving is real but
**too small to resolve** against a live scene: predicted ~3.5% of a core,
observed 1.75 points against a run-to-run spread of 13. Costmap rate is
unchanged. Keep the cache; do not quote a CPU number for it on the lean stack.

## Setup

| | |
|---|---|
| car | dinosaur, rebooted 15:54 immediately before the test |
| cameras | lean launch, `cameras:=front,left,right costmap:=false`, all three steady at 8.00 Hz from the first run to the last |
| config | `perception_dinosaur.yaml`, `publish_rate` 10 Hz |
| nocache build | `~/chris_test/ab/nocache`, `04312ca` (lean-launch branch) |
| cache build | `~/chris_test/ab/cache`, `bc95360` (same, plus the cache) |
| difference | `diff -r` over the package: **one file**, `costmap_node.py` |
| runner | `~/chris_test/ab_cache.sh`, 60 s measured per run after 45 s warm-up |
| CPU | `/proc/<pid>/stat` utime+stime delta for the costmap node only |

Both builds ran against **one set of cameras that was never restarted**, so the
only variable is the node under test. The `/perception/known` guard (`19b58c5`)
is deliberately *not* in either build — it also touches `costmap_node.py` and
would have made this a two-variable comparison.

## Results

Pass 1 — A/B/A:

| variant | costmap Hz | CPU | seg computed/cached | depth/ipm | inference sub/rep/done |
|---|---|---|---|---|---|
| nocache | 9.860 | 102% | — | 718/186 | 449/6/442 |
| cache | 10.023 | 95% | 1197/303 | 186/307 | 447/2/445 |
| nocache | 9.852 | 94% | — | 86/333 | 443/0/443 |

Pass 2 — interleaved, to average out drift:

| variant | costmap Hz | CPU | seg computed/cached | depth/ipm | inference sub/rep/done |
|---|---|---|---|---|---|
| cache | 9.821 | 89% | 1200/300 | 73/329 | 446/0/446 |
| nocache | 9.834 | 93% | — | 88/324 | 447/0/447 |
| cache | 9.640 | 93% | 1198/302 | 58/311 | 450/0/450 |
| nocache | 9.723 | 95% | — | 117/321 | 451/0/450 |
| cache | 9.865 | 100% | 1199/301 | 582/200 | 448/3/446 |
| nocache | 9.983 | 96% | — | 375/344 | 456/0/457 |

Means over all nine runs:

| | costmap Hz | CPU | segmentations skipped |
|---|---|---|---|
| nocache (5 runs) | 9.850 | 96.0% | — |
| cache (4 runs) | 9.837 | 94.25% | **20.1%** |

## What the numbers say

**The hit rate is exactly right, and it is stable.** 300-303 cached of 1500
camera-frame presentations, in all four cache runs, with no run deviating by
more than 3. That is the arithmetic working out: cameras publish at 8 Hz, the
tick runs at 10 Hz, so 2 ticks in every 10 are handed a frame they have already
processed. The predicted number was ~20% and the measured number is 20.1%.

**The CPU saving is below the noise floor, and that was predictable.**
Segmentation plus the white-line mask is ~18 ms of a ~28 ms tick (P2, laptop
profile). Skipping 20% of it saves ~3.6 ms per tick, which at 9.85 Hz is
**~3.5% of one core**. Measured difference: 1.75 points, same sign. Run-to-run
spread for the *same* build: 89-102%, i.e. 13 points. The effect is a quarter
of the noise.

**The noise is the scene, not the machine.** Depth projections per window
ranged from 58 to 718 across runs while the inference count barely moved
(443-456). Depth projection is the expensive path; the IPM warp is the cheap
fallback. So whenever something in view produced depth-usable detections, the
node's CPU rose regardless of which build was running — run 5 (cache, 100%) had
582 depth projections, the most of any run except the very first.

**No regression anywhere.** Costmap rate 9.837 vs 9.850 Hz, inference
submitted/replaced/completed effectively identical, `replaced` at 0-6 in both.

## Conclusion

Keep the cache. It does precisely what it was written to do, it is deterministic
(keyed on the frame's stamp, dropped by `/perception/reset`), and it costs
nothing measurable. But **the lean stack is the wrong place to look for its
payoff** — the same change was worth 53% of segmentations on the competition
preset, where the tick is slow enough that each frame is presented several
times. On the lean stack it is a 20% saving on 18 ms of a 100 ms budget.

To actually resolve the ~3.5% would need the scene removed as a variable:
replay a recorded bag into both builds, or time the segmentation stage directly
rather than sampling whole-process CPU. Neither is worth doing for a 3.5%
saving that is already understood analytically.

## Camera lessons from three failed attempts earlier the same day

All three earlier attempts produced `costmapHz=?` and zero counters. None of it
was the node.

1. **Tear the boot stack down in the right order.** `percept-stack`'s
   `ExecStop` is `tmux kill-session`, a hard kill. Stopping the service (or
   killing the tmux session) while the ZEDs are streaming leaves Argus holding
   streams, and every later open degrades: cameras that stream for about a
   minute and then sit idle at ~8% CPU, with `BadParameter` and
   `NvPclOpen: PCL Open Failed` in nvargus. Order that worked: kill the
   per-camera watchdog loops, `camera_doctor.sh --stop` (SIGINT + Argus wait),
   *then* stop the service.
2. **Hold the camera subscriptions open for the whole session.** Between
   variants the costmap node exits, and nothing was subscribed to the cameras.
   `chris_test/hold_cams.py` keeps one raw (undeserialised) subscription on all
   nine topics, so the stream never has to be re-established for the next
   variant. It cost 15.1% of a core, identically for both builds.
   Whether the ZED wrapper genuinely fails to resume after its subscriber count
   reaches zero, or whether that was only the wedged Argus, is **still not
   settled** — the holder makes the question moot for testing.
3. **nvargus error counts include boot noise.** A healthy stack right after
   boot already shows 30 `AlreadyAllocated` / 45 `BadParameter` in the last ten
   minutes while all three cameras stream at 24-33% CPU. Judge cameras by
   whether frames arrive, not by the error count.
4. **Abort if a costmap node is already running.** The first attempt crashed
   part-way through and left an orphan node subscribed; the next two runs then
   shared the cameras with it. `ab_cache.sh` now refuses to start in that state.
5. **`pgrep -f <pattern>` matches the shell running it.** Killing "every
   watchdog" over SSH killed the SSH session too. Use a bracket pattern
   (`[w]hile true; do ...`).

Related: `camera_doctor`'s verdict ranked `AlreadyAllocated` above the
sensor-open failures and so reported HELD — "no restart or reboot helps" — with
all three drivers down and nothing holding a camera. Fixed in `19b58c5`; the
car's copy is still the old build.
