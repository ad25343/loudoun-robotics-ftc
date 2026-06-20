/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.DashboardBridge;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.LoopProfiler;

/**
 * Demonstrates live telemetry to FTC Dashboard.
 *
 * Run this OpMode, then open <b>http://192.168.43.1:8080/dash</b> from any
 * browser on the same WiFi as the Control Hub. You should see live charts of:
 * <ul>
 *   <li>{@code sine} — a 1 Hz sine wave (sanity check that data is flowing)</li>
 *   <li>{@code gp1_left_y}, {@code gp1_right_x} — gamepad sticks in real time</li>
 *   <li>{@code loop_ms}, {@code voltage_v}, {@code memory_mb} — from {@link LoopProfiler}</li>
 * </ul>
 *
 * No drive hardware required. Just the Control Hub.
 *
 * <h3>If the dashboard is empty</h3>
 * <ol>
 *   <li>Check {@code DashboardBridge.isAvailable()} — printed to driver-station
 *       telemetry. If false, the dependency isn't on the classpath.</li>
 *   <li>Make sure your browser is on the same WiFi as the Control Hub
 *       (usually the hub's hotspot SSID like {@code 1234-RC}).</li>
 *   <li>The URL is {@code http://192.168.43.1:8080/dash} on Control Hub,
 *       {@code http://192.168.49.1:8080/dash} on Expansion Hub mode.</li>
 * </ol>
 */
@TeleOp(name = "LR Sample: Dashboard Live", group = "Loudoun")
public class SampleDashboardLive extends LinearOpMode {

    @Override
    public void runOpMode() {
        LoopProfiler health = new LoopProfiler(hardwareMap);

        telemetry.addData("FTC Dashboard available", DashboardBridge.isAvailable());
        telemetry.addLine("Press PLAY. Then open http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive()) {
            health.tick();

            // Send a few live values per loop.
            double sine = Math.sin(t.seconds() * 2 * Math.PI);  // 1 Hz sine
            DashboardBridge.send("sine",        sine);
            DashboardBridge.send("gp1_left_y",  -gamepad1.left_stick_y);   // inverted for "up = positive"
            DashboardBridge.send("gp1_right_x",  gamepad1.right_stick_x);
            DashboardBridge.send("seconds",      t.seconds());

            // LoopProfiler also sends loop_ms / voltage_v / memory_mb to the dashboard.
            health.addTelemetry(telemetry);

            // Flush the buffered dashboard packet once per loop.
            DashboardBridge.flush();
            telemetry.update();
        }
    }
}
