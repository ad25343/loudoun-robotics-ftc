/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * One-line setup for REV Control Hub bulk reads.
 *
 * Why this matters: by default, every call to {@code motor.getCurrentPosition()},
 * {@code sensor.getDistance()}, etc. triggers a separate I2C/CAN round-trip.
 * Five sensor reads in one loop = five round-trips = slow loop. Bulk caching
 * collapses all reads in one loop into ONE round-trip, often 2-3x faster.
 *
 * Most rookie teams don't know this exists. Use this helper.
 *
 * Two modes:
 *   AUTO   — cache auto-refreshes when ANY hardware read happens after the
 *            previous read. Set-and-forget. Use this unless you have a reason not to.
 *   MANUAL — cache only refreshes when you call {@link #clearAll()}. Slightly faster
 *            but you MUST call clearAll() at the top of every loop.
 *
 * Usage (AUTO mode, the default):
 * <pre>
 *   new BulkCache(hardwareMap);   // that's it
 * </pre>
 *
 * Usage (MANUAL mode, for max performance):
 * <pre>
 *   BulkCache cache = new BulkCache(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
 *   while (opModeIsActive()) {
 *       cache.clearAll();   // top of every loop
 *       // ... read sensors / motor positions / etc.
 *   }
 * </pre>
 */
public class BulkCache {

    private final List<LynxModule> hubs;

    /** AUTO caching mode — set and forget. Recommended for most teams. */
    public BulkCache(HardwareMap hwMap) {
        this(hwMap, LynxModule.BulkCachingMode.AUTO);
    }

    /** Explicit caching mode. Use MANUAL for max performance + call {@link #clearAll()} every loop. */
    public BulkCache(HardwareMap hwMap, LynxModule.BulkCachingMode mode) {
        hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(mode);
        }
    }

    /** Force cache refresh — only needed in MANUAL mode. Call at the top of each loop. */
    public void clearAll() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
