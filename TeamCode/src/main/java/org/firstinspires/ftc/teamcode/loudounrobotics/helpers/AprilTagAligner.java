/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Wraps the FTC SDK's {@link AprilTagProcessor} to give your OpMode
 * "find a specific AprilTag and tell me how to drive to it."
 *
 * Uses the SDK's built-in AprilTag pipeline — no OpenCV pipeline of your own
 * needed. Plug into a {@code VisionPortal.Builder} as a processor; call
 * {@link #getDetection()} or {@link #getDriveCorrections(double)} from your
 * OpMode loop.
 *
 * Coordinate system (FTC SDK convention):
 *   • {@code ftcPose.range}   — straight-line distance to the tag (inches)
 *   • {@code ftcPose.bearing} — left/right angle from the robot to the tag
 *                                (degrees, + = right, − = left)
 *   • {@code ftcPose.yaw}     — robot's rotation relative to the tag normal
 *                                (degrees, + = facing too far right, − = left)
 *
 * Usage — basic detection:
 * <pre>
 *   AprilTagAligner aligner = new AprilTagAligner();
 *   aligner.setTargetTag(5);   // only care about tag ID 5
 *
 *   VisionPortal portal = new VisionPortal.Builder()
 *       .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
 *       .addProcessor(aligner.getProcessor())
 *       .build();
 *
 *   while (opModeIsActive() &amp;&amp; aligner.isDetected()) {
 *       AprilTagDetection tag = aligner.getDetection();
 *       telemetry.addData("Range",   tag.ftcPose.range);
 *       telemetry.addData("Bearing", tag.ftcPose.bearing);
 *       telemetry.update();
 *   }
 * </pre>
 *
 * Usage — drive to a tag at 12" range:
 * <pre>
 *   while (opModeIsActive()) {
 *       AprilTagAligner.DriveCorrections c = aligner.getDriveCorrections(12.0);
 *       if (c == null) { drive.stop(); continue; }   // tag lost
 *
 *       // Scale each axis through a P-controller (tune the gains for your robot)
 *       double fwd    = clamp(c.forward * 0.04, -0.4, 0.4);
 *       double strafe = clamp(c.strafe  * 0.025, -0.4, 0.4);
 *       double turn   = clamp(c.turn    * 0.015, -0.4, 0.4);
 *
 *       drive.driveRobotCentric(fwd, strafe, turn);
 *
 *       if (Math.abs(c.forward) &lt; 1.0 &amp;&amp; Math.abs(c.strafe) &lt; 1.0 &amp;&amp; Math.abs(c.turn) &lt; 2.0) {
 *           drive.stop(); break;
 *       }
 *   }
 * </pre>
 */
public class AprilTagAligner {

    private final AprilTagProcessor processor;
    private int targetTagId = -1;  // -1 = match any tag

    public AprilTagAligner() {
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
    }

    /** The underlying AprilTag processor — pass to {@code VisionPortal.Builder.addProcessor()}. */
    public AprilTagProcessor getProcessor() {
        return processor;
    }

    /** Restrict detection to a specific tag ID. Pass {@code -1} to accept any tag. */
    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    /** Most recent matching detection, or null if none. */
    public AprilTagDetection getDetection() {
        List<AprilTagDetection> detections = processor.getDetections();
        if (detections == null || detections.isEmpty()) return null;
        for (AprilTagDetection det : detections) {
            if (targetTagId < 0 || det.id == targetTagId) {
                return det;
            }
        }
        return null;
    }

    /** True if a matching tag is in view this frame. */
    public boolean isDetected() {
        return getDetection() != null;
    }

    /**
     * Returns drive corrections (forward / strafe / turn) to move toward the
     * target tag at the given range, or null if no tag is detected.
     *
     * Each component is the RAW ERROR in its natural unit:
     *   • {@code forward} — inches to close (positive = drive forward)
     *   • {@code strafe}  — bearing degrees to cancel (positive = strafe right)
     *   • {@code turn}    — yaw degrees to cancel (positive = turn clockwise)
     *
     * Multiply each by a small gain (≈ 0.01–0.05) before feeding to the drive.
     * For a more disciplined approach, wrap each in a {@link PIDFController}.
     */
    public DriveCorrections getDriveCorrections(double targetRangeInches) {
        AprilTagDetection det = getDetection();
        if (det == null || det.ftcPose == null) return null;
        return new DriveCorrections(
                det.ftcPose.range - targetRangeInches,
                det.ftcPose.bearing,
                det.ftcPose.yaw
        );
    }

    public static class DriveCorrections {
        public final double forward;
        public final double strafe;
        public final double turn;

        public DriveCorrections(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }
}
