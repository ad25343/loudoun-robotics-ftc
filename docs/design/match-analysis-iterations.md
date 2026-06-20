# Match Data Analysis — Design Iterations

## Origin: why we even tried

FTC teams burn motors and servos. Robots brown out mid-match. Drivers feel performance degrade over 2:30 of play. Coaches have ~6 minutes between matches to swap a battery, fix what broke, and queue for the next round — but they're guessing what to fix, because they can't see what happened inside the robot during the previous match.

Our first attempt was to *prevent* the failures with proactive software protection:

- `MotorGuard` / `StallGuard` — auto-cut motor power when commanded power was high but the encoder wasn't moving
- `BatteryManager` — voltage compensation, throttling under load, refuse-to-start pre-match gates
- `ServoCare` — auto-release on endpoint dwell

**Three rounds of adversarial design review killed all of them.** The verdict was unified:

> *"False-positives cost a match (lose seeding for the day). True-positives save $35 and 10 minutes in the pit. Teams will always optimize for the match. Helpful safety code that triggers wrongly during a scoring cycle gets ripped out at the first regional."*

That feedback forced the pivot. If we couldn't prevent failures without making them worse, we'd help the coach *understand* them, fast enough that the next match wasn't a repeat. The design target collapsed to one sentence:

> **In under 30 seconds, the coach should know what went wrong and what to change before the next match.**

Every iteration below was a step toward that target.

## Iteration 1 — FTC Dashboard (wrong moment)

**Proposed:** Use FTC Dashboard (existing community tool) — live telemetry web UI on the Control Hub's IP.

**Killed because:** *"This is not an in-game thing. This is post-game analysis."* Dashboard solves live tuning, not post-match forensics.

**Lesson:** Identify the *moment* the tool is used, not just the data it touches.

## Iteration 2 — Web-based replay viewer

**Proposed:** Static HTML at `loudounrobotics.org/match-replay`. Drop a CSV → voltage chart, loop time chart, anomaly panel.

**Forward question:** What does the CSV actually need to contain? Defined a column schema (required: `time_s`, `phase`, `voltage_v`, `loop_ms`; recommended: per-motor `power`/`position`/`amps`; nice-to-have: gamepad input, heading).

## Iteration 3 — Reduce logging boilerplate (`MatchRecorder`)

**Problem:** Logging 14 fields = `log.writeRow(a, b, c, ..., n)` every loop. Verbose, error-prone, hard to evolve.

**Proposed:** Builder pattern — `.withMotor("intake", intakeMotor)` + one `snapshot()` call per loop.

**Changed because:** FTC reality — auto and teleop are *two separate OpModes*. Match-ID stitching overcomplicated it.

## Iteration 4 — Generalize to `Recorder` with lambdas

**Coach correction:** *"Each motor and servo is declared separately in the OpMode. The recorder shouldn't try to wrap them."*

**Revised:** Lambda registration — Recorder reads from suppliers, doesn't own hardware.

```java
Recorder rec = new Recorder("teleop")
    .add("voltage_v", () -> health.voltage())
    .add("intake_pwr", () -> intakeMotor.getPower());
rec.record(telemetry);  // CSV + driver-station mirror in one call
```

**Persona reviewer found flaws:**

- Auto-flooding the driver station kills the driver's curated debug screen
- `telemetry.update()` inside `record()` races with the OpMode's own updates
- One supplier exception kills the whole recorder

**Hardened:** opt-in `.mirror(names...)`, rate-limit DS mirror to 10 Hz, never call `telemetry.update()`, try/catch each supplier.

## Iteration 5 — Drop the action-taking helpers (revisited)

**Earlier proposal:** `intake.isStalled()`, `battery.lowVoltage()` convenience methods.

**Why we revisited:** Persona review had already killed `MotorGuard`/`StallGuard`/`BatteryManager`. Reintroducing them as "lightweight checks" was the same bad idea in a smaller package.

**Decision:** Teams write inline checks against raw `LoopProfiler.voltage()` and motor positions. Don't ship abstractions that get torn out.

## Iteration 6 — How does the CSV leave the Control Hub?

**Proposed:** Replay HTML hosted on the Control Hub via FTC Dashboard's web server. Driver Station phone navigates to `192.168.43.1:8080/replay`.

**Persona reviewer rejected:**

- Drivers won't type URLs between matches (3-min pit window, focused on battery + queue)
- Driver Station is sacred — refs side-eye browser tabs on competition hardware
- "Most recent CSV" picks the wrong file 50% of the time (practice runs + auto-only tests + real match interleaved)
- Wi-Fi-Direct is the robot control channel — bandwidth contention with next match
- Dashboard's file API has no auth — shared event Wi-Fi = data leak surface

**Lesson:** **"Files survive. Workflows die."** Infrastructure dependent on browser tabs, URLs, or competition-day timing gets ripped out.

## Iteration 7 — Coach correction on Dashboard scope

*"FTC Dashboard? We cannot compete with them. We are building a post-game analysis tool."*

**Decision:** Drop the FTC Dashboard dependency entirely. Different tool, different scope.

## Iteration 8 — Locking the device model

| Device | Role |
|---|---|
| **Control Hub** | On robot. Runs OpMode. Stores CSVs in `/sdcard/FIRST/`. |
| **Driver Station** | Drivers' Android controller. Never touched for analysis. |
| **Team laptop** | Reads CSVs via REV Hardware Client / USB. The analysis device. |

## Iteration 9 — `summary.txt` isn't actionable enough

**Initial idea:** Recorder writes 5-line summary on close — duration, min voltage, max loop time, anomaly counts.

**Coach pushback:** *"The summary text does not tell me what happened and how to prep for the next match."*

**Revised format — diagnostic narratives + concrete actions:**

```
✗ INTAKE JAMMED at 1:42
  Intake motor commanded full power for 0.4 s with the encoder stuck.
  Voltage dropped 12.1 V → 9.8 V.
  → BEFORE NEXT MATCH: clear intake, inspect chain alignment.

✗ BATTERY SAG accelerated after 1:30
  Sustained below 11 V for the final 14 s.
  → ACTION: swap battery now.
```

**Lesson:** Numbers ≠ insights. Coaches need *narratives + actions*.

## Iteration 10 — Schema challenge: non-standard column names

**Coach observation:** *"Not everyone uses standard names."* `intake_pwr` vs `int_power` vs `IntakePower` — the analyzer can't infer meaning from strings alone.

**Solution — typed `Recorder` API.** Builder methods bake the *meaning* into the schema:

```java
Recorder rec = new Recorder("teleop")
    .voltage(() -> health.voltage())                    // type: VOLTAGE
    .loopMs(() -> health.lastLoopMs())                  // type: LOOP_MS
    .motor("intake", intakeMotor)                       // type: MOTOR
    .servo("gripper", gripperServo)                     // type: SERVO
    .gamepad(1, gamepad1)                               // type: GAMEPAD
    .custom("vision_zone", () -> detector.getZone());   // free-form, not auto-analyzed
```

CSV gets a schema header line:

```
# schema: voltage_v=VOLTAGE, intake_power=MOTOR_POWER:intake, intake_position=MOTOR_POSITION:intake
time_s, voltage_v, intake_power, intake_position, ...
```

Analyzer reads types → applies rules → produces insights regardless of column naming.

## Iteration 11 — APK vs PWA (intermediate)

**Coach preference:** Native Android app on a tablet — pick a CSV, get insights.

**Reality check:** Both APK and PWA share the same file-transfer constraint (competition Wi-Fi-Direct excludes the coach's phone from the Control Hub network).

| | APK | PWA (installable web app) |
|---|---|---|
| Coach experience | Tap icon, pick file | Tap icon (after one-time "install"), pick file |
| Updates | Re-sideload each release | Auto-update on next online visit |
| Maintenance | Heavy (Android changes every season) | Light (one codebase) |
| Coach's actual workflow | Identical | Identical |

**Decision: PWA.** Same UX, drastically less to maintain. APK deferred as a possible second shell on the same analyzer logic.

## Iteration 12 — Final adversarial review kills `summary.txt` and the 30-second target

We sent the locked architecture (Recorder + `summary.txt` + PWA) back to the persona reviewer one more time. They killed two pieces of it.

**The "30-second target" is a fantasy.**

> *"Between matches at a qualifier I have 4-7 minutes total, and 3 of those are queueing. Reading summary.txt on a laptop tethered to the Control Hub via REV Hardware Client, with one student panicking and one parent asking about lunch, is not a 30-second activity."*

Coaches don't open a laptop between matches. They might open one at home, with brain cycles. The PWA at home is the artifact that survives the season; `summary.txt` is the piece a coach "pretends to use for a week then forgets about."

**Two analyzers is the smell we shouldn't ignore.**

If we ship `summary.txt` (Java analyzer on Control Hub) AND the PWA (JS analyzer in browser), they WILL drift the first time someone tweaks a threshold in JS and forgets the Java side — exact same failure mode as the action-taking helpers we already killed.

**`finally { rec.close() }` — implementation note.** The reviewer suggested moving close to a `stop()` override because finally blocks can be skipped on interrupt. We tried that and discovered `LinearOpMode.stop()` is `final` and cannot be overridden. The reliable cleanup hook for LinearOpMode is `try / finally` around the loop: the framework signals stop by interrupting the thread, and the resulting `InterruptedException` unwinds through finally normally. The only case finally doesn't run is a hard JVM kill, where buffered data is lost regardless.

**Logger-induced loop spikes are real.** SD-card writes on Control Hubs can cause 30 ms hiccups. If the log says "LOOP SPIKE at 0:23" and the spike was the logger itself, the team will tune away from a working camera config for nothing.

### Six specific fixes to the Recorder

1. **Default-mirror `voltage_v` + `loop_ms`** to the Driver Station. Opt-OUT, not opt-in. ("If one kid forgets `.mirror()`, we drive blind.")
2. **Add a `logger_ms` column** that records how long `record()` itself took, so the analyzer can subtract logger cost before flagging loop spikes.
3. **Move `close()` to the OpMode's `stop()` override** — `finally` won't reliably run on force-stop.
4. **Time-based flush** every 250 ms wall-clock, not every 50 rows. Bounds data loss regardless of loop rate.
5. **Robot ID in filename.** Multi-robot teams get CSV collisions; read robot ID from a `robot.id` file on the Hub (or pass it explicitly).
6. **Tolerate truncated CSVs** in the analyzer — drop a half-written final line gracefully.

### Defer `summary.txt`

Defer until BOTH conditions are met:
- A shared declarative rules file (JSON or YAML) exists, so the Java and JS analyzers can't drift
- We've watched 3 real events to confirm coaches actually open REV Hardware Client between matches

If after 3 events no coach has used it, kill it permanently.

---

## Final design (post-Iteration-12 revision)

### Components (revised — two, not three)

| Component | Status | What it does |
|---|---|---|
| **`Recorder` helper** | ✅ Ship | Typed builder API. Writes CSV with schema header. Defaults to mirroring `voltage_v` + `loop_ms` to the Driver Station. Time-based flush. Self-times every loop into a `logger_ms` column. Cleanup runs in the OpMode's `stop()` override, not `finally`. |
| **Match-replay PWA** | ✅ Ship | `loudounrobotics.org/match-replay`. Drag CSV in. Reads schema header. Applies type-based rules. Charts + insights. Installable to home screen. The artifact that survives the season. |
| `summary.txt` generator | ⏸ Deferred | Only ships once (a) a shared declarative rules file exists so Java + JS analyzers can't drift, AND (b) we've observed a coach actually use it across 3 real events. Otherwise it's dead weight. |

### End-to-end flow

1. **Before match (home):** team writes OpMode with `Recorder`, uploads to Control Hub.
2. **During match:** OpMode logs CSV to `/sdcard/FIRST/`. Driver sees `voltage_v` + `loop_ms` on the DS by default.
3. **Match ends:** Recorder's `close()` (from OpMode `stop()` override) flushes the buffered writer.
4. **At home:** team pulls all CSVs (REV Hardware Client / `adb pull`), opens the PWA, gets full charts + insights for deeper analysis.

### Why we dropped `summary.txt` for now

The "30-second pit-window glance at a diagnostic narrative" was a fantasy. Coaches at qualifiers have 4-7 minutes between matches, mostly spent queueing. They don't open laptops between matches. They open them at home with brain cycles — and at home, the PWA is the better tool.

---

## Engineering lessons captured

1. **The real problem hides behind the technical one.** We started thinking about "data extraction friction" but the actual problem was *"coaches lose matches because they can't diagnose failures fast enough."*
2. **Identify the moment, not just the data.** Live tools and post-match tools have completely different design constraints.
3. **Files survive. Workflows die.** Infrastructure that depends on browser tabs, URLs, or competition-day timing gets ripped out.
4. **Adversarial review is gold.** Four rounds of persona review killed six over-engineered designs (MotorGuard, StallGuard, BatteryManager, in-pit replay viewer, DS auto-mirror, and finally `summary.txt`) that would have either hurt teams or drifted out of sync within a season.
5. **Schema flexibility cuts both ways.** Free-form column names look flexible but make analysis impossible. Typed registration captures intent.
6. **Numbers aren't insights** — but **the place to render insights is wherever the coach has brain cycles**, not wherever the data lives.
7. **One source of truth or none.** Shipping two implementations of the same detection logic (Java analyzer + JS analyzer) guarantees they drift. Either build a shared declarative rules layer, or ship just one shell.
8. **Test what the lifecycle actually guarantees.** `try { ... } finally { ... }` is not equivalent to Android's `stop()` callback when the OpMode is force-stopped. Resource cleanup hooks go where the framework owns the lifecycle.
