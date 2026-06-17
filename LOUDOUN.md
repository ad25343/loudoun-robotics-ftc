# Loudoun Robotics — FTC Starter

This is [Loudoun Robotics](https://www.loudounrobotics.org)' fork of the official [FIRST-Tech-Challenge/FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController). It's a starting point for rookie FTC teams in Loudoun County, Virginia.

## What's added

Everything LR adds lives under one package:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/loudounrobotics/
├── helpers/    — small reusable utilities (debounced gamepad, etc.)
└── examples/   — sample OpModes that use the helpers
```

Nothing else in this repo is touched. You can pull upstream SDK updates each season without merge conflicts.

## How to use it

1. Clone this repo and open it in Android Studio (Ladybug 2024.2 or newer)
2. Write your team's OpModes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` — same as you would on the official SDK
3. Use the helpers in `loudoun/helpers/` when they save you boilerplate (or don't — they're optional)
4. Look at `loudoun/examples/` to see how each helper is used

## Licensing

This repo has two licenses:

- **Upstream FTC SDK code** — [BSD 3-Clause](LICENSE) (inherited from FIRST-Tech-Challenge)
- **LR additions in `loudoun/`** — [MIT](LICENSE-LR-ADDITIONS)

If you fork, modify, or copy LR's helpers, the MIT license applies — keep the copyright notice, do what you want.

## Resources

- [loudounrobotics.org](https://www.loudounrobotics.org) — what Loudoun Robotics is
- [Python Academy](https://www.loudounrobotics.org/python-academy.html) — our free SPIKE Prime curriculum (for the K-8 crowd before they hit FTC)
- [FTC Documentation](https://ftc-docs.firstinspires.org/) — official FIRST docs
- [FTC Javadoc](https://javadoc.io/doc/org.firstinspires.ftc) — SDK API reference

## Contributing

Not open for outside contributions yet — this is in early development. If you're an LR coach or mentor and want to add a helper, open an issue or email contact@loudounrobotics.org.
