/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Thin convenience wrapper around {@link FtcDashboard}.
 *
 * FTC Dashboard is the de-facto competitive-team telemetry tool — once the
 * dependency is on the classpath (see {@code build.dependencies.gradle}), open
 * <b>http://192.168.43.1:8080/dash</b> from any browser on the robot's WiFi
 * to see live graphs of whatever values you send.
 *
 * <h2>Two ways to use it</h2>
 *
 * <h3>1. Quick: send one or more named values per loop</h3>
 * <pre>
 *   DashboardBridge.send("voltage", 12.4);
 *   DashboardBridge.send("loop_ms", 18.2);
 *   DashboardBridge.send("heading_deg", 47.5);
 * </pre>
 * Each call buffers a value. To actually push the packet to the browser, call
 * {@link #flush()} once per loop iteration (typically at the end of your loop):
 * <pre>
 *   DashboardBridge.flush();
 * </pre>
 *
 * <h3>2. Full control: build your own packet</h3>
 * <pre>
 *   TelemetryPacket pkt = new TelemetryPacket();
 *   pkt.put("range_in", range);
 *   pkt.fieldOverlay().setStroke("red").strokeCircle(target.x, target.y, 3);
 *   FtcDashboard.getInstance().sendTelemetryPacket(pkt);
 * </pre>
 *
 * <h2>Failure handling</h2>
 *
 * If FTC Dashboard isn't on the classpath at runtime or its instance can't be
 * created, every method here silently no-ops. Your OpMode keeps running — you
 * just don't get live charts. Same crash-safety guarantee as {@link CsvLogger}.
 *
 * <h2>Recommended workflow</h2>
 *
 * <ol>
 *   <li>Live tuning while you drive: {@code DashboardBridge.send(...)} + {@code flush()}</li>
 *   <li>Post-match analysis: pull CSVs off {@code /sdcard/FIRST/} via the dashboard's
 *       file browser, then drop them into the LR Match Replay viewer (planned
 *       at loudounrobotics.org/match-replay) for charts and anomaly detection.</li>
 * </ol>
 */
public final class DashboardBridge {

    private static TelemetryPacket pending = new TelemetryPacket();
    private static boolean dashboardAvailable = checkDashboard();

    private DashboardBridge() { /* utility class */ }

    private static boolean checkDashboard() {
        try {
            // Touch the singleton early so a missing dependency fails here, not in a hot loop.
            return FtcDashboard.getInstance() != null;
        } catch (Throwable t) {
            return false;
        }
    }

    /**
     * Buffer a value to send on the next {@link #flush()}.
     * @param key   chart label (e.g. "voltage", "loop_ms")
     * @param value any value — primitives, strings, anything {@code TelemetryPacket} accepts
     */
    public static synchronized void send(String key, Object value) {
        if (!dashboardAvailable) return;
        try {
            pending.put(key, value);
        } catch (Throwable t) {
            // Disable on first failure so we don't keep allocating packets.
            dashboardAvailable = false;
        }
    }

    /**
     * Flush the buffered values to the dashboard. Call once per loop iteration,
     * typically at the end of your loop right before {@code telemetry.update()}.
     */
    public static synchronized void flush() {
        if (!dashboardAvailable) return;
        try {
            FtcDashboard.getInstance().sendTelemetryPacket(pending);
        } catch (Throwable t) {
            dashboardAvailable = false;
        } finally {
            pending = new TelemetryPacket();
        }
    }

    /** True if FTC Dashboard is present and reachable. */
    public static boolean isAvailable() {
        return dashboardAvailable;
    }
}
