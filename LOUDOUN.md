# Loudoun Robotics — FTC Starter

This is [Loudoun Robotics](https://www.loudounrobotics.org)' fork of the official [FIRST-Tech-Challenge/FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController). It's a starting point for rookie FTC teams in Loudoun County, Virginia.

## Status

These helpers **build cleanly against FTC SDK v11.1** but **have not yet been tested on a real robot**. First teams to use them should expect minor surprises — open an issue or email contact@loudounrobotics.org if you hit one.

## What's added

Everything LR adds lives under one package:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/
├── helpers/    — reusable utilities (see index below)
└── examples/   — sample OpModes that demonstrate the helpers
```

Nothing else in this repo is touched — you can pull upstream SDK updates each season without merge conflicts.

## Helpers index

| Helper | What it does |
|---|---|
| [GamepadEx](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/GamepadEx.java) | Debounced gamepad — fires `wasJustPressed` once per press instead of every loop |
| [MecanumDrive](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/MecanumDrive.java) | Robot-centric + field-centric mecanum with the canonical denominator math |
| [BulkCache](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/BulkCache.java) | One-line REV hub bulk-read setup; 2–3× loop-speed win |
| [EncoderHoming](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/EncoderHoming.java) | Drive a motor at low power until a limit switch (TouchSensor or DigitalChannel) trips, then zero the encoder |
| [PIDFController](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/PIDFController.java) | Plain-Java PID + feedforward for arms, slides, drive distance, heading hold |
| [CsvLogger](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/CsvLogger.java) | Crash-safe RFC-4180 CSV logging to `/sdcard/FIRST/` |
| [LoopProfiler](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/LoopProfiler.java) | Loop time (last/avg/max) + battery voltage + JVM memory |
| [ThreeZoneDetector](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/ThreeZoneDetector.java) | "Where's the prop?" via HSV color — works for red/blue/yellow; sensitive to lighting |
| [ChrominanceZoneDetector](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/ChrominanceZoneDetector.java) | Same three-zone API but uses YCrCb chrominance — lighting-robust; red and blue only |
| [AprilTagAligner](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/AprilTagAligner.java) | Find an AprilTag by ID; returns drive corrections to align with it |
| [VisionPipelineBase](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/helpers/VisionPipelineBase.java) | (Advanced) abstract base for shape-detection pipelines — subclass and override `analyze()` |

## Examples

| Example | Helpers used |
|---|---|
| [SampleTeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleTeleOp.java) | GamepadEx |
| [SampleMecanumTeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleMecanumTeleOp.java) | MecanumDrive + BulkCache + GamepadEx |
| [SampleHealthLogging](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleHealthLogging.java) | LoopProfiler + CsvLogger + BulkCache |
| [SampleZoneDetection](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleZoneDetection.java) | ThreeZoneDetector |
| [SampleCompositeAuto](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/examples/SampleCompositeAuto.java) | 6 helpers together — vision-driven mecanum auto with heading hold + CSV logging |

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
