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
 * Usage — drive to a tag at 12" range. <b>Tune the gains for your robot:</b>
 * start at 0.01 on each axis, double until the robot just begins to oscillate
 * near the target, then halve. There is no one-size-fits-all default.
 * <pre>
 *   double kForward = 0.01;   // tune
 *   double kStrafe  = 0.01;   // tune
 *   double kTurn    = 0.01;   // tune
 *
 *   while (opModeIsActive()) {
 *       AprilTagAligner.DriveCorrections c = aligner.getDriveCorrections(12.0);
 *       if (c == null) { drive.stop(); continue; }   // tag lost
 *
 *       // Apply deadband on each axis so we don't hunt at small errors
 *       double fwd    = Math.abs(c.forward) &lt; 0.5 ? 0 : clamp(c.forward * kForward, -0.4, 0.4);
 *       double strafe = Math.abs(c.strafe)  &lt; 0.5 ? 0 : clamp(c.strafe  * kStrafe,  -0.4, 0.4);
 *       double turn   = Math.abs(c.turn)    &lt; 2.0 ? 0 : clamp(c.turn    * kTurn,    -0.4, 0.4);
 *
 *       drive.driveRobotCentric(fwd, strafe, turn);
 *
 *       if (fwd == 0 &amp;&amp; strafe == 0 &amp;&amp; turn == 0) { drive.stop(); break; }
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
     * <h3>Axis assignment — this is the important part</h3>
     *
     * <ul>
     *   <li>{@code forward} — inches between current range and target range
     *       (positive = drive forward to close).</li>
     *   <li>{@code strafe}  — inches of lateral offset from {@code ftcPose.x}
     *       (positive = strafe right to center on the tag).</li>
     *   <li>{@code turn}    — degrees of bearing (positive = turn clockwise
     *       to face the tag). Bearing drives <em>turn</em>, not strafe.</li>
     * </ul>
     *
     * Common bug: assigning bearing to strafe makes a mecanum drive crab sideways
     * while still pointing the wrong direction. The correct intuition is "turn
     * to face it, strafe to center on it, drive forward to close the range."
     *
     * <h3>Gains — tune for your robot</h3>
     *
     * Start each axis at gain ≈ 0.01 and double until the robot just begins to
     * oscillate near the target, then halve. The right value depends on robot
     * mass, gearing, and friction — there is no one-size-fits-all default.
     *
     * Add a deadband on each axis (e.g. forward ±0.5", strafe ±0.5", turn ±2°)
     * to avoid hunting on small errors.
     */
    public DriveCorrections getDriveCorrections(double targetRangeInches) {
        AprilTagDetection det = getDetection();
        if (det == null || det.ftcPose == null) return null;
        return new DriveCorrections(
                det.ftcPose.range - targetRangeInches,  // forward: close the range
                det.ftcPose.x,                          // strafe:  cancel lateral offset
                det.ftcPose.bearing                     // turn:    point at the tag
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
