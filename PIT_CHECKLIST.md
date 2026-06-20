# Pit Checklist — Between Matches

Software helpers can't catch every burnout. Most fried motors in FTC are
spotted by **a coach who looked, touched, and listened** before the next
match. Use this checklist between every match.

## 60-second walk-around

| ✅ | Check | What you're looking for |
|---|---|---|
| ☐ | **Touch each motor** | Warm = fine. Hot enough that you let go quickly = problem. A motor at 60-70°C is on the path to dying. |
| ☐ | **Touch each servo gearbox** | Same test. Servos that hold against a load get hot fastest. |
| ☐ | **Listen to each servo at rest** | A faint hum = current draw at idle = mechanical bind. Loud whine = death is imminent. |
| ☐ | **Wiggle every motor wire at the hub** | Loose JST connector → intermittent loss of motor under match stress. |
| ☐ | **Chain tension** | Press a chain at the middle of its span. Should deflect ~1/8 inch. Loose = skipping; tight = wearing teeth. |
| ☐ | **Wheel bolts** | Tighten any that move. Mecanum wheels and intake wheels are the usual culprits. |
| ☐ | **Battery voltage at rest** | Should read **≥ 13.0V** with a multimeter (NOT the driver-station gauge). Anything lower → swap. |
| ☐ | **Battery wire crimps** | Loose crimps on the Anderson connector cause brownouts under load. Squeeze and look for movement. |
| ☐ | **Camera lens** | Any smudge, fingerprint, or dust = autonomous failure. Wipe with a microfiber. |
| ☐ | **Phone tether (if used)** | USB cable seated; phone has battery; phone is on the right network. |

## After a hard match (collision, push battle)

| ✅ | Check | What you're looking for |
|---|---|---|
| ☐ | **Frame integrity** | Visually inspect for bent extrusion or loose screws on the corners. |
| ☐ | **Wheel alignment** | Robot should track straight when pushed across the floor. If it veers, a wheel/axle shifted. |
| ☐ | **All hub ports** | Reseat any visibly stressed connector. |
| ☐ | **Re-zero encoders** | If a slip clutch slipped or you pushed a slide past its limit, the encoder is now wrong. Re-home in init. |

## Match-by-match log

A small post-match note pays off in seeding rankings. Write down:

- **Match #**, **alliance**, **partner team**
- **Score** (yours + opponent)
- **What worked** (one line)
- **What broke or felt off** (one line)
- **Battery # used** (helps you spot a bad battery)
- **Driver fatigue** (1–5; you'll see patterns after 8 matches)

## Software safety can't do these

The LR helpers catch some classes of failure (loop spikes via `LoopProfiler`,
CSV records via `CsvLogger`, post-match analysis via FTC Dashboard). But
the categories below are *physical*, and your eyes/ears/hands are the only
detector:

- Hot motor (no temp sensor on FTC motors)
- Loose connector (intermittent — won't show as a steady fault)
- Bent frame (no software signal)
- Chain about to skip (no software signal until it does)
- Servo gearbox stripping (audible before electrical)
- Battery on its last cycle (voltage reads OK at rest, sags under load)

If you remember one thing: **touch every motor between every match.**
