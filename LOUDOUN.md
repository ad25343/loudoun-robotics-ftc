# Loudoun Robotics — FTC Starter

This is [Loudoun Robotics](https://www.loudounrobotics.org)' fork of the official [FIRST-Tech-Challenge/FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController). It's a starting point for rookie FTC teams in Loudoun County, Virginia.

## Status

These helpers **build cleanly against FTC SDK v11.1** and have been **adversarially code-reviewed** by simulated FTC personas (driver / engineer / coach) with the top issues addressed. They still **have not been tested on a real robot.** First teams to use them should expect minor surprises — open an issue or email contact@loudounrobotics.org if you hit one.

Known caveats per helper are listed in the "Tested status" column below.

## What's added

Everything LR adds lives under one package:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/
├── helpers/    — reusable utilities (see index below)
└── examples/   — sample OpModes that demonstrate the helpers
```

Nothing else in this repo is touched — you can pull upstream SDK updates each season without merge conflicts.

## Helpers index

| Helper | What it does | Tested status |
|---|---|---|
| [GamepadEx](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/GamepadEx.java) | Debounced gamepad — fires `wasJustPressed` once per press instead of every loop | ✓ Build clean, untested on hardware |
| [MecanumDrive](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/MecanumDrive.java) | Robot-centric + field-centric mecanum with the canonical denominator math; `setMaxOutputScale()` reserves headroom for voltage compensation | ⚠ Strafe-correction factor is per-robot (default 1.1 is folk wisdom); measure yours by driving a diagonal |
| [BulkCache](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/BulkCache.java) | One-line REV hub bulk-read setup (defaults to MANUAL — call `clearAll()` per loop); 2–3× loop-speed win | ✓ Build clean, untested on hardware |
| [EncoderHoming](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/EncoderHoming.java) | Drive a motor at low power until a limit switch (TouchSensor or DigitalChannel) trips, then zero the encoder | ✓ Build clean, untested on hardware |
| [PIDFController](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/PIDFController.java) | Plain-Java PID + velocity-FF (`kF`) + constant-FF (`kS`) for arms, slides, drive distance, heading hold; integral cap defaults to 1.0 (mitigates windup) | ⚠ Gains must be tuned per mechanism; defaults to ±1.0 output range |
| [CsvLogger](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/CsvLogger.java) | Crash-safe RFC-4180 CSV logging to `/sdcard/FIRST/`; 8 KB buffered, flushes every 50 rows or every 1 s | ⚠ Verify scoped-storage permission on your RC app version |
| [LoopProfiler](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/LoopProfiler.java) | Loop time (last/avg/max) + battery voltage + JVM memory; auto-mirrors to FTC Dashboard if present | ✓ Build clean, untested on hardware |
| [DashboardBridge](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/DashboardBridge.java) | Thin wrapper around FTC Dashboard for live web telemetry charts at `http://192.168.43.1:8080/dash` | ⚠ Verify dashboard is reachable on your control hub's WiFi |
| [ThreeZoneDetector](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/ThreeZoneDetector.java) | "Where's the prop?" via HSV color — works for red (dual-range OR'd for wraparound), blue, yellow; sensitive to lighting | ⚠ Tune pixel threshold at the venue (lighting varies) |
| [ChrominanceZoneDetector](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/ChrominanceZoneDetector.java) | Same three-zone API but uses YCrCb chrominance — lighting-robust; red and blue only; auto-scales pixel-sum threshold to frame size | ⚠ Default threshold 170 may need tuning for high-saturation props |
| [ColorShape](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/ColorShape.java) | Find shapes (triangle/square/circle/etc.) of a target color — color-mask first, then classify. Call `close()` in OpMode `stop()` to release Mats | ⚠ Circle detection uses circularity + vertex count; tune for your shapes |
| [AprilTagAligner](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/AprilTagAligner.java) | Find an AprilTag by ID; returns drive corrections (forward / strafe / turn) — bearing drives TURN, lateral offset drives STRAFE | ⚠ Gains start at 0.01, tune up to oscillation, then halve |

## Examples

| Example | Helpers used |
|---|---|
| [SampleTeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleTeleOp.java) | GamepadEx |
| [SampleMecanumTeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleMecanumTeleOp.java) | MecanumDrive + BulkCache + GamepadEx |
| [SampleHealthLogging](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleHealthLogging.java) | LoopProfiler + CsvLogger + BulkCache |
| [SampleZoneDetection](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleZoneDetection.java) | ThreeZoneDetector |
| [SampleShapeDetection](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleShapeDetection.java) | ColorShape — live detection telemetry for tuning |
| [SampleCompositeAuto](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleCompositeAuto.java) | 6 helpers together — vision-driven mecanum auto with heading hold + CSV logging |
| [SampleDashboardLive](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleDashboardLive.java) | DashboardBridge + LoopProfiler — live charts in the browser via FTC Dashboard |

## Getting data off the Control Hub

After a match you'll want to look at what the robot saw and did. Three paths from easiest to hardest:

### 1. FTC Dashboard (recommended)
With the dashboard dependency on the classpath (it ships with this repo), open
**`http://192.168.43.1:8080/dash`** from any browser on the Control Hub's WiFi.

- Live charts of every value sent through {@code DashboardBridge.send(...)} during the match
- Built-in file browser → download `/sdcard/FIRST/*.csv` with one click
- Works on a laptop in the pit, a phone, anything with a browser

### 2. REV Hardware Client
The official REV desktop app. Slower, but no extra setup required.

- Connect the Control Hub via USB
- Browse files under `/sdcard/FIRST/`
- Right-click → download

### 3. ADB (advanced)
For teams comfortable with the Android Debug Bridge.

```bash
adb pull /sdcard/FIRST/
```

### Coming soon: LR Match Replay viewer
Drop a CSV into **loudounrobotics.org/match-replay** (planned) for interactive charts and auto-detected anomalies ("voltage hit 9.8V at 1:42 — likely a stall on the slide motor"). Builds on the standard {@code CsvLogger} format.

## How to use it

1. Clone this repo and open it in Android Studio (Ladybug 2024.2 or newer)
2. Write your team's OpModes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` — same as you would on the official SDK
3. Import helpers as you need them: `import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.MecanumDrive;`
4. Look at the matching `examples/` file to see how each helper is wired up
5. The OpModes in `examples/` appear in your Driver Station under the "Loudoun" group — try them with the matching hardware config

## Licensing

This repo has two licenses:

- **Upstream FTC SDK code** — [BSD 3-Clause](LICENSE) (inherited from FIRST-Tech-Challenge)
- **LR additions in `loudounrobotics/`** — [MIT](LICENSE-LR-ADDITIONS)

If you fork, modify, or copy LR's helpers, the MIT license applies — keep the copyright notice, do what you want.

## Resources

- [loudounrobotics.org](https://www.loudounrobotics.org) — what Loudoun Robotics is
- [Python Academy](https://www.loudounrobotics.org/python-academy.html) — our free SPIKE Prime curriculum (the on-ramp before FTC for K–8 students)
- [FTC Documentation](https://ftc-docs.firstinspires.org/) — official FIRST docs
- [FTC Javadoc](https://javadoc.io/doc/org.firstinspires.ftc) — SDK API reference
- [Game Manual 0](https://gm0.org/) — community-maintained FTC strategy + programming guide

## Contributing

Not open for outside contributions yet — this is in early development. If you're an LR coach or mentor and want to add a helper, open an issue or email contact@loudounrobotics.org.
