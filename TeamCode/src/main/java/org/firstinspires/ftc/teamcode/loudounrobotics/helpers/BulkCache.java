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
 *   MANUAL (default) — cache only refreshes when you call {@link #clearAll()}.
 *                      Deterministic, fastest in real teams. You MUST call
 *                      {@code clearAll()} at the top of every loop.
 *   AUTO            — cache silently re-reads when the same sensor is read again
 *                      after a previous read in the same loop. Set-and-forget but
 *                      slower than MANUAL once a team queries the same sensor in
 *                      more than one place (which happens fast as helpers stack up).
 *
 * Usage (MANUAL — the recommended pattern):
 * <pre>
 *   BulkCache cache = new BulkCache(hardwareMap);
 *   while (opModeIsActive()) {
 *       cache.clearAll();   // top of every loop
 *       // ... read sensors / motor positions / etc.
 *   }
 * </pre>
 *
 * Usage (AUTO — only when you don't control the loop):
 * <pre>
 *   new BulkCache(hardwareMap, LynxModule.BulkCachingMode.AUTO);   // set and forget
 * </pre>
 */
public class BulkCache {

    private final List<LynxModule> hubs;

    /**
     * MANUAL caching mode. Call {@link #clearAll()} at the top of every loop.
     * This is what competitive FTC teams use; it's deterministic and faster than AUTO
     * once helpers stack up.
     */
    public BulkCache(HardwareMap hwMap) {
        this(hwMap, LynxModule.BulkCachingMode.MANUAL);
    }

    /** Explicit caching mode. Most teams should stick with the default MANUAL constructor. */
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
