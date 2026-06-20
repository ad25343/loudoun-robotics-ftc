/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Match-day health monitor: loop time, battery voltage, and JVM memory.
 *
 * Why this matters:
 *   • A robot that runs fine in practice but bricks at minute 2 is usually
 *     burning through battery faster than expected — voltage tells you that.
 *   • A robot that misses control inputs is usually running a slow loop —
 *     loop time tells you that.
 *   • Memory creep means you're allocating Mats / arrays / objects every
 *     loop without releasing them — memory tells you that.
 *
 * Call {@link #tick()} once per loop. Call {@link #addTelemetry(Telemetry)}
 * when you want to see the numbers on the driver station.
 *
 * Usage:
 * <pre>
 *   LoopProfiler health = new LoopProfiler(hardwareMap);
 *
 *   while (opModeIsActive()) {
 *       health.tick();
 *
 *       // ... do robot stuff ...
 *
 *       health.addTelemetry(telemetry);
 *       telemetry.update();
 *   }
 * </pre>
 */
public class LoopProfiler {

    private final VoltageSensor voltageSensor;  // may be null on a robot with no hub

    private long lastTickNs = -1;
    private long loopCount = 0;
    private double lastLoopMs = 0;
    private double totalLoopMs = 0;
    private double minLoopMs = Double.MAX_VALUE;
    private double maxLoopMs = 0;

    public LoopProfiler(HardwareMap hwMap) {
        VoltageSensor vs = null;
        try {
            vs = hwMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            // No voltage sensor (extremely unusual on a real robot, but allowed)
        }
        voltageSensor = vs;
    }

    /** Call once per loop iteration. Tracks loop time. */
    public void tick() {
        long nowNs = System.nanoTime();
        if (lastTickNs > 0) {
            lastLoopMs = (nowNs - lastTickNs) / 1_000_000.0;
            totalLoopMs += lastLoopMs;
            loopCount++;
            if (lastLoopMs < minLoopMs) minLoopMs = lastLoopMs;
            if (lastLoopMs > maxLoopMs) maxLoopMs = lastLoopMs;
        }
        lastTickNs = nowNs;
    }

    public double lastLoopMs() { return lastLoopMs; }
    public double avgLoopMs()  { return loopCount > 0 ? totalLoopMs / loopCount : 0; }
    public double minLoopMs()  { return loopCount > 0 ? minLoopMs : 0; }
    public double maxLoopMs()  { return maxLoopMs; }
    public long   loopCount()  { return loopCount; }

    /** Battery voltage in volts. Returns -1 if no voltage sensor is present. */
    public double voltage() {
        return voltageSensor != null ? voltageSensor.getVoltage() : -1;
    }

    /** JVM memory currently in use, in megabytes. */
    public long memoryUsedMB() {
        Runtime r = Runtime.getRuntime();
        return (r.totalMemory() - r.freeMemory()) / 1_048_576L;
    }

    /**
     * Drop a compact health block into your driver station telemetry — and, if
     * FTC Dashboard is on the classpath, also send the values to live dashboard
     * charts (see {@link DashboardBridge}). The dashboard send is no-op if the
     * dashboard isn't reachable.
     *
     * Note: this method does NOT call {@code DashboardBridge.flush()}. Flush
     * yourself once per loop, after all your send() calls.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Loop ms (last / avg / max)",
                "%.1f / %.1f / %.1f", lastLoopMs(), avgLoopMs(), maxLoopMs());
        if (voltage() > 0) {
            telemetry.addData("Voltage", "%.2f V", voltage());
        }
        telemetry.addData("Memory used", "%d MB", memoryUsedMB());

        // Mirror to FTC Dashboard for live graphs. No-ops if dashboard isn't present.
        DashboardBridge.send("loop_ms", lastLoopMs());
        DashboardBridge.send("loop_ms_max", maxLoopMs());
        if (voltage() > 0) DashboardBridge.send("voltage_v", voltage());
        DashboardBridge.send("memory_mb", memoryUsedMB());
    }
}
