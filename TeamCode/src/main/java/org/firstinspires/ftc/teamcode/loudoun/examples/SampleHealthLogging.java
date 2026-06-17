/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudoun.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.loudoun.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.loudoun.helpers.CsvLogger;
import org.firstinspires.ftc.teamcode.loudoun.helpers.LoopProfiler;

/**
 * Minimal example combining {@link LoopProfiler} and {@link CsvLogger}.
 *
 * Does no driving — just sits and logs robot health to /sdcard/FIRST/
 * for the duration of the match. Use as a diagnostic when troubleshooting:
 *   • Loop time spikes during a match
 *   • Voltage sag during heavy mechanism use
 *   • Memory leaks across long-running OpModes
 *
 * No hardware required besides a Control Hub. Pull the CSV afterwards via:
 *   adb pull /sdcard/FIRST/
 * or the REV Hardware Client.
 */
@TeleOp(name = "LR Sample: Health Logging", group = "Loudoun")
public class SampleHealthLogging extends LinearOpMode {

    @Override
    public void runOpMode() {
        new BulkCache(hardwareMap);

        LoopProfiler health = new LoopProfiler(hardwareMap);
        CsvLogger log = new CsvLogger("health");
        log.writeHeader("time_s", "loop_ms", "voltage_v", "memory_mb");

        ElapsedTime matchTimer = new ElapsedTime();

        telemetry.addLine("Press PLAY to start logging.");
        telemetry.addData("Output file", log.getFilename());
        telemetry.addData("Logging", log.isOpen() ? "OK" : "DISABLED (file open failed)");
        telemetry.update();

        waitForStart();
        matchTimer.reset();

        while (opModeIsActive()) {
            health.tick();

            log.writeRow(
                    String.format("%.3f", matchTimer.seconds()),
                    String.format("%.2f", health.lastLoopMs()),
                    String.format("%.2f", health.voltage()),
                    health.memoryUsedMB()
            );

            health.addTelemetry(telemetry);
            telemetry.addData("Match time", "%.1f s", matchTimer.seconds());
            telemetry.addData("Rows logged", health.loopCount());
            telemetry.update();
        }

        log.close();
    }
}
