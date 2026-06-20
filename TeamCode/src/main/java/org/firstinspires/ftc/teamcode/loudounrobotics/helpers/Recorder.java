/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Typed per-loop telemetry recorder. Writes a CSV with a schema header line that the
 * match-replay analyzer at <a href="https://www.loudounrobotics.org/match-replay">
 * loudounrobotics.org/match-replay</a> reads to render charts and detect anomalies.
 *
 * <h2>Usage</h2>
 *
 * Register data sources once at OpMode init using typed builder methods. Each typed
 * method bakes the FIELD MEANING (voltage, motor power, gamepad stick, etc.) into a
 * schema metadata block so the analyzer recognizes it regardless of column naming.
 *
 * <pre>
 *   public class TeleOpFoo extends LinearOpMode {
 *       &#64;Override
 *       public void runOpMode() {
 *           LoopProfiler health = new LoopProfiler(hardwareMap);
 *           MecanumDrive drive = new MecanumDrive(hardwareMap, "fl", "bl", "fr", "br", "imu");
 *           DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
 *
 *           Recorder rec = new Recorder("teleop")
 *               .voltage(health::voltage)              // type: VOLTAGE
 *               .loopMs(health::lastLoopMs)            // type: LOOP_MS
 *               .heading(() -&gt; Math.toDegrees(drive.getHeading()))
 *               .motor("intake", intake)               // adds power+position+amps
 *               .gamepad(1, gamepad1);                 // adds canonical sticks+buttons
 *
 *           waitForStart();
 *           try {
 *               while (opModeIsActive()) {
 *                   health.tick();
 *                   // robot logic...
 *                   rec.record(telemetry);             // CSV row + DS mirror
 *               }
 *           } finally {
 *               rec.close();                           // flushes the buffer
 *           }
 *       }
 *   }
 * </pre>
 *
 * <h2>Lifecycle: why {@code close()} goes in {@code try / finally}</h2>
 *
 * {@code LinearOpMode.stop()} is {@code final} — you cannot override it. The framework
 * signals stop by interrupting the OpMode thread; the thrown {@code InterruptedException}
 * (or a normal {@code opModeIsActive() == false} exit) unwinds through the finally block,
 * running cleanup. The only way to lose the finally block is a hard JVM kill — and at
 * that point buffered data is gone anyway.
 *
 * <h2>Driver Station mirroring</h2>
 *
 * By default, {@code voltage_v} and {@code loop_ms} are mirrored to the Driver Station
 * automatically — the driver always sees those two numbers. Use {@link #mirror(String...)}
 * to add more fields, or {@link #noMirror()} to disable the defaults. Mirroring is
 * rate-limited internally to 10 Hz so it doesn't flood the DS packet stream.
 *
 * Important: {@code record(telemetry)} adds data items but DOES NOT call
 * {@code telemetry.update()} — your OpMode owns that call. This lets you add your own
 * warnings (battery low, intake stalled, etc.) between {@code record()} and {@code update()}.
 *
 * <h2>Self-timing</h2>
 *
 * Every row includes a {@code logger_ms} column with the time {@code record()} itself
 * took. This lets the analyzer subtract logger cost before flagging loop spikes —
 * otherwise SD-card write hiccups masquerade as robot-logic spikes.
 *
 * <h2>Crash safety</h2>
 *
 * Writes go through an 8 KB {@link CsvLogger} buffer that flushes every 250 ms
 * wall-clock (not per row). On force-stop, you lose at most 250 ms of data. Each
 * registered supplier is wrapped in try/catch — one NPE writes {@code "ERR"} into
 * that cell and the recorder keeps going.
 */
public class Recorder {

    private static final long FLUSH_INTERVAL_NS  = 250_000_000L;   // 250 ms
    private static final long MIRROR_INTERVAL_NS = 100_000_000L;   // 10 Hz

    private final CsvLogger log;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final List<Entry> entries = new ArrayList<>();
    private final Set<String> mirroredNames = new LinkedHashSet<>();

    private boolean headerWritten = false;
    private long lastFlushNanos = 0;
    private long lastMirrorNanos = 0;

    /** Logs to /sdcard/FIRST/&lt;baseName&gt;_&lt;timestamp&gt;.csv. */
    public Recorder(String baseName) {
        this(baseName, null);
    }

    /**
     * Logs to /sdcard/FIRST/&lt;baseName&gt;_&lt;robotId&gt;_&lt;timestamp&gt;.csv.
     *
     * Pass a robot ID when your team runs multiple robots (V1, V2, etc.) so their CSVs
     * don't collide in the analyzer's "latest" picker.
     */
    public Recorder(String baseName, String robotId) {
        String name = (robotId != null && !robotId.isEmpty())
                ? baseName + "_" + robotId
                : baseName;
        this.log = new CsvLogger(name);
    }

    // ===== Typed registration =====

    /** Battery voltage in volts. Auto-mirrored to the Driver Station. */
    public Recorder voltage(DoubleSupplier source) {
        entries.add(new Entry("voltage_v", "VOLTAGE", source::getAsDouble));
        mirroredNames.add("voltage_v");
        return this;
    }

    /** Loop time in milliseconds. Auto-mirrored to the Driver Station. */
    public Recorder loopMs(DoubleSupplier source) {
        entries.add(new Entry("loop_ms", "LOOP_MS", source::getAsDouble));
        mirroredNames.add("loop_ms");
        return this;
    }

    /** Robot heading in degrees. */
    public Recorder heading(DoubleSupplier source) {
        entries.add(new Entry("heading_deg", "HEADING", source::getAsDouble));
        return this;
    }

    /**
     * Register a motor. Adds {@code motor_&lt;name&gt;_power} and
     * {@code motor_&lt;name&gt;_position}. If the motor is {@code DcMotorEx}, also adds
     * {@code motor_&lt;name&gt;_amps}.
     */
    public Recorder motor(String name, DcMotor motor) {
        String prefix = "motor_" + name;
        entries.add(new Entry(prefix + "_power",    "MOTOR_POWER:" + name,    motor::getPower));
        entries.add(new Entry(prefix + "_position", "MOTOR_POSITION:" + name, () -> (double) motor.getCurrentPosition()));
        if (motor instanceof DcMotorEx) {
            DcMotorEx ex = (DcMotorEx) motor;
            entries.add(new Entry(prefix + "_amps", "MOTOR_CURRENT:" + name, () -> ex.getCurrent(CurrentUnit.AMPS)));
        }
        return this;
    }

    /** Register a servo. Adds {@code servo_&lt;name&gt;_pos}. */
    public Recorder servo(String name, Servo servo) {
        entries.add(new Entry("servo_" + name + "_pos", "SERVO_POSITION:" + name, servo::getPosition));
        return this;
    }

    /**
     * Register a gamepad. Adds {@code gp&lt;n&gt;_lx}, {@code _ly}, {@code _rx}, {@code _ry},
     * and {@code _buttons} (bitmask of pressed buttons + triggers).
     *
     * Stick Y is inverted from the raw SDK value so positive = forward — matches what
     * humans expect when looking at a recording.
     */
    public Recorder gamepad(int index, Gamepad gp) {
        String prefix = "gp" + index;
        entries.add(new Entry(prefix + "_lx",      "GAMEPAD:" + index + ":lx",      () -> (double)  gp.left_stick_x));
        entries.add(new Entry(prefix + "_ly",      "GAMEPAD:" + index + ":ly",      () -> (double) -gp.left_stick_y));
        entries.add(new Entry(prefix + "_rx",      "GAMEPAD:" + index + ":rx",      () -> (double)  gp.right_stick_x));
        entries.add(new Entry(prefix + "_ry",      "GAMEPAD:" + index + ":ry",      () -> (double) -gp.right_stick_y));
        entries.add(new Entry(prefix + "_buttons", "GAMEPAD:" + index + ":buttons", () -> (double)  encodeButtons(gp)));
        return this;
    }

    /** Generic numeric field. Analyzer treats it as CUSTOM (no built-in rules). */
    public Recorder add(String name, DoubleSupplier source) {
        entries.add(new Entry(name, "CUSTOM", source::getAsDouble));
        return this;
    }

    /** Generic text/enum field. Analyzer treats it as CUSTOM. */
    public Recorder addText(String name, Supplier<Object> source) {
        entries.add(new Entry(name, "CUSTOM", source));
        return this;
    }

    // ===== Mirror configuration =====

    /** Add fields to the Driver Station mirror set (in addition to the default voltage + loop_ms). */
    public Recorder mirror(String... names) {
        for (String n : names) mirroredNames.add(n);
        return this;
    }

    /** Clear all mirrors. Even voltage and loop_ms will stop showing on the Driver Station. */
    public Recorder noMirror() {
        mirroredNames.clear();
        return this;
    }

    // ===== Per-loop call =====

    /**
     * Pull a fresh value from every registered source, write one CSV row, and mirror
     * the configured fields to the Driver Station (rate-limited to 10 Hz).
     *
     * Does NOT call {@code telemetry.update()} — your OpMode owns that.
     *
     * @param telemetry the OpMode's telemetry (may be null if you don't want mirroring)
     */
    public void record(Telemetry telemetry) {
        long recordStart = System.nanoTime();

        if (!headerWritten) {
            writeSchemaAndHeader();
            headerWritten = true;
        }

        // Pull values, with try/catch per supplier so one ERR doesn't kill the whole row.
        Object[] values = new Object[entries.size()];
        for (int i = 0; i < entries.size(); i++) {
            try {
                values[i] = entries.get(i).source.get();
            } catch (Exception e) {
                values[i] = "ERR";
            }
        }

        // Driver Station mirror — rate-limited.
        long now = System.nanoTime();
        if (telemetry != null
                && !mirroredNames.isEmpty()
                && (now - lastMirrorNanos) >= MIRROR_INTERVAL_NS) {
            for (int i = 0; i < entries.size(); i++) {
                if (mirroredNames.contains(entries.get(i).name)) {
                    telemetry.addData(entries.get(i).name, values[i]);
                }
            }
            lastMirrorNanos = now;
        }

        // Time-based flush (wall-clock, not row-count).
        if ((now - lastFlushNanos) >= FLUSH_INTERVAL_NS) {
            log.flush();
            lastFlushNanos = now;
        }

        // Build the row: time_s, ...values, logger_ms
        Object[] row = new Object[entries.size() + 2];
        row[0] = String.format(java.util.Locale.US, "%.3f", matchTimer.seconds());
        System.arraycopy(values, 0, row, 1, values.length);
        long recordEnd = System.nanoTime();
        row[row.length - 1] = String.format(java.util.Locale.US, "%.2f",
                (recordEnd - recordStart) / 1e6);

        log.writeRow(row);
    }

    /**
     * Flush + close. Call from a {@code try / finally} around your OpMode loop —
     * {@code LinearOpMode.stop()} is final, so finally is the cleanup hook.
     */
    public void close() {
        log.close();
    }

    /** Absolute path of the CSV file being written. */
    public String getFilename() {
        return log.getFilename();
    }

    // ===== Internals =====

    private void writeSchemaAndHeader() {
        // Schema line — bypasses CSV escaping; analyzer reads any line starting with '#'.
        StringBuilder schema = new StringBuilder("# schema: time_s=TIME");
        for (Entry e : entries) {
            schema.append(", ").append(e.name).append("=").append(e.typeMetadata);
        }
        schema.append(", logger_ms=LOGGER_MS");
        log.writeComment(schema.toString());

        // Column header row — standard RFC-4180.
        String[] cols = new String[entries.size() + 2];
        cols[0] = "time_s";
        for (int i = 0; i < entries.size(); i++) cols[i + 1] = entries.get(i).name;
        cols[cols.length - 1] = "logger_ms";
        log.writeHeader(cols);
    }

    private static int encodeButtons(Gamepad gp) {
        int b = 0;
        if (gp.a)             b |= 1;
        if (gp.b)             b |= 2;
        if (gp.x)             b |= 4;
        if (gp.y)             b |= 8;
        if (gp.dpad_up)       b |= 16;
        if (gp.dpad_down)     b |= 32;
        if (gp.dpad_left)     b |= 64;
        if (gp.dpad_right)    b |= 128;
        if (gp.left_bumper)   b |= 256;
        if (gp.right_bumper)  b |= 512;
        if (gp.left_trigger  > 0.5) b |= 1024;
        if (gp.right_trigger > 0.5) b |= 2048;
        if (gp.start)         b |= 4096;
        if (gp.back)          b |= 8192;
        return b;
    }

    private static final class Entry {
        final String name;
        final String typeMetadata;
        final Supplier<?> source;

        Entry(String name, String typeMetadata, Supplier<?> source) {
            this.name = name;
            this.typeMetadata = typeMetadata;
            this.source = source;
        }
    }
}
