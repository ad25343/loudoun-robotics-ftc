/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.ThreeZoneDetector;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Autonomous example: detect which of three zones the game prop is in,
 * then act on it.
 *
 * Uses {@link ThreeZoneDetector} with the red-element factory.
 * Change to {@link ThreeZoneDetector#forBlueElement()} or
 * {@link ThreeZoneDetector#forYellowElement()} for other colors —
 * or instantiate directly with custom HSV bounds.
 *
 * Hardware required:
 *   • WebcamName "Webcam 1" — any USB webcam mounted on the robot
 *
 * Tuning tips (if detection isn't reliable):
 *   1. Run this OpMode with the prop in each zone, one at a time
 *   2. Read the "Zone sums" telemetry — left/middle/right pixel-sums
 *   3. Set the threshold to about half the lowest sum you saw for a
 *      "real" detection: {@code detector.setThreshold(...)}
 *   4. If the prop barely registers, widen the HSV range — instantiate
 *      ThreeZoneDetector with custom Scalar bounds
 */
@Autonomous(name = "LR Sample: Zone Detection", group = "Loudoun")
public class SampleZoneDetection extends LinearOpMode {

    @Override
    public void runOpMode() {
        ThreeZoneDetector detector = ThreeZoneDetector.forRedElement();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .build();

        // Pre-start: stream telemetry so you can position the prop and calibrate
        while (!isStarted() && !isStopRequested()) {
            double[] sums = detector.getZoneSums();
            telemetry.addData("Detection (live)", detector.getDetection());
            telemetry.addData("Zone sums",
                    "L=%.0f  M=%.0f  R=%.0f", sums[0], sums[1], sums[2]);
            telemetry.addLine();
            telemetry.addLine("Position the prop, then press PLAY.");
            telemetry.addLine("If detection is unstable, raise the threshold in code.");
            telemetry.update();
        }

        if (isStopRequested()) return;

        ThreeZoneDetector.ZonePosition where = detector.getDetection();
        portal.close();  // free the webcam — we don't need vision during auto driving

        telemetry.addData("Locked detection", where);
        telemetry.update();

        switch (where) {
            case LEFT:
                // Drive to the left scoring zone
                break;
            case MIDDLE:
                // Drive to the middle scoring zone
                break;
            case RIGHT:
                // Drive to the right scoring zone
                break;
            case NONE:
                // Fall back to a default path
                break;
        }

        // ... rest of autonomous routine
    }
}
