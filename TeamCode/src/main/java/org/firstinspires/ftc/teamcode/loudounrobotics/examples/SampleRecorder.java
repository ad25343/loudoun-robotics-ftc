/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.LoopProfiler;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.Recorder;

/**
 * Demonstrates {@link Recorder} with the correct lifecycle:
 *
 * <ul>
 *   <li>Register data sources once at init using typed builder methods</li>
 *   <li>Call {@code rec.record(telemetry)} once per loop</li>
 *   <li>Add your own warnings between {@code record()} and {@code telemetry.update()}</li>
 *   <li>Close in a {@code try / finally} around the loop — that's the LinearOpMode pattern</li>
 * </ul>
 *
 * <b>Why {@code try / finally} works here:</b> {@link LinearOpMode#stop()} is final and can't be
 * overridden. The framework signals stop by interrupting the OpMode thread; the thrown
 * {@code InterruptedException} (or a normal {@code opModeIsActive() == false} exit) unwinds
 * through the finally block, running cleanup. The only way to lose the finally block is a hard
 * JVM kill — and at that point buffered data is gone anyway.
 *
 * <p>This OpMode requires no hardware besides a Control Hub. It just logs voltage and loop
 * time. Use it to sanity-check that {@code /sdcard/FIRST/recorder_demo_*.csv} is written,
 * has a schema header line, and can be opened in the match-replay viewer.
 */
@TeleOp(name = "LR Sample: Recorder", group = "Loudoun")
public class SampleRecorder extends LinearOpMode {

    @Override
    public void runOpMode() {
        new BulkCache(hardwareMap);

        LoopProfiler health = new LoopProfiler(hardwareMap);

        // Pass a robot ID if you run multiple bots — keeps their CSVs from colliding
        // in the analyzer's "latest" picker. Skip if you only have one robot.
        Recorder rec = new Recorder("recorder_demo")
                .voltage(health::voltage)         // auto-mirrors to Driver Station
                .loopMs(health::lastLoopMs)       // auto-mirrors to Driver Station
                .gamepad(1, gamepad1);            // logs sticks + buttons

        telemetry.addLine("Press PLAY. Voltage + loop_ms will mirror to this screen.");
        telemetry.addData("Log file", rec.getFilename());
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                health.tick();

                // Robot logic would go here.

                rec.record(telemetry);   // CSV row + DS mirror; does NOT call update()

                // Add your own warnings AFTER record(), BEFORE update(). These won't be in
                // the CSV — they're driver-facing only. The team owns the warning logic.
                if (health.voltage() < 11.0) telemetry.addLine("BATTERY LOW");
                if (health.lastLoopMs() > 50) telemetry.addLine("LOOP SPIKE");

                telemetry.update();
            }
        } finally {
            // Flushes the buffered writer. Always runs on a clean exit, interrupt-based
            // stop, or thrown exception — the only way to skip it is a hard JVM kill.
            rec.close();
        }
    }
}
