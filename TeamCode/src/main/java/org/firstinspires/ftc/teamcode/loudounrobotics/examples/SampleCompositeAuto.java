/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.CsvLogger;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.LoopProfiler;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.MecanumDrive;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.ThreeZoneDetector;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.ThreeZoneDetector.ZonePosition;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * End-to-end autonomous demonstrating how the LR helpers compose:
 *
 *   1. {@link BulkCache}        — keeps the loop fast (set-and-forget)
 *   2. {@link MecanumDrive}     — robot-centric driving
 *   3. {@link ThreeZoneDetector} — find the prop position before start
 *   4. {@link PIDFController}   — heading-hold during the drive (keeps the
 *                                  robot pointing forward even if a wheel slips)
 *   5. {@link LoopProfiler}     — track loop time + voltage during the run
 *   6. {@link CsvLogger}        — log every loop to /sdcard/FIRST/ for analysis
 *
 * Routine:
 *   PRE-START — stream live zone detection so you can position the prop
 *               and verify the detector locks on
 *   START     — lock the zone reading and close the camera
 *   PHASE 1   — drive forward for 1.5 s with heading hold
 *   PHASE 2   — strafe LEFT, MIDDLE (no strafe), or RIGHT based on the
 *               detected zone for 1.0 s
 *   STOP      — flush the log
 *
 * Hardware required:
 *   • Mecanum drive: DcMotor "fl" "bl" "fr" "br"
 *   • IMU: "imu" (REV Control Hub built-in)
 *   • Webcam: WebcamName "Webcam 1"
 *
 * Note: this is a DEMONSTRATION of helper composition, not a tuned auto
 * routine. The drive durations are arbitrary; the PID gains assume a
 * lightweight robot. Tune for your specific build.
 */
@Autonomous(name = "LR Sample: Composite Auto", group = "Loudoun")
public class SampleCompositeAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        new BulkCache(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap,
                "fl", "bl", "fr", "br", "imu");
        ThreeZoneDetector detector = ThreeZoneDetector.forRedElement();
        LoopProfiler health = new LoopProfiler(hardwareMap);
        CsvLogger log = new CsvLogger("composite_auto");
        log.writeHeader("time_s", "phase", "zone", "heading_deg",
                        "loop_ms", "voltage_v");

        // Heading-hold PID: 0 = "stay pointed at the start direction"
        PIDFController headingPID = new PIDFController(0.02, 0, 0.003, 0);
        headingPID.setOutputLimits(-0.5, 0.5);
        headingPID.setSetpoint(0);  // hold initial heading

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .build();

        // PRE-START — stream live detection for calibration
        while (!isStarted() && !isStopRequested()) {
            double[] sums = detector.getZoneSums();
            telemetry.addLine("=== PRE-START ===");
            telemetry.addData("Detected zone", detector.getDetection());
            telemetry.addData("Zone sums (L/M/R)",
                    "%.0f / %.0f / %.0f", sums[0], sums[1], sums[2]);
            telemetry.addLine();
            telemetry.addLine("Press PLAY to lock detection and start.");
            telemetry.update();
        }

        if (isStopRequested()) {
            portal.close();
            log.close();
            return;
        }

        ZonePosition zone = detector.getDetection();
        portal.close();  // free camera bandwidth

        ElapsedTime matchTimer = new ElapsedTime();

        // PHASE 1 — drive forward 1.5 s, holding heading
        runPhase("forward", 1.5, zone, drive, headingPID, health, log, matchTimer,
                () -> 0.4,   // forward power
                () -> 0.0);  // no strafe

        // PHASE 2 — strafe based on detected zone for 1.0 s
        final double strafePower;
        switch (zone) {
            case LEFT:   strafePower = -0.4; break;
            case RIGHT:  strafePower =  0.4; break;
            default:     strafePower =  0.0; break;
        }
        runPhase("strafe_" + zone, 1.0, zone, drive, headingPID, health, log, matchTimer,
                () -> 0.0,
                () -> strafePower);

        drive.stop();
        log.close();

        telemetry.addLine("=== DONE ===");
        telemetry.addData("Final zone", zone);
        telemetry.addData("Log file", log.getFilename());
        telemetry.update();
    }

    /** Runs one driving phase, holding heading via PID. */
    private void runPhase(String name, double durationSeconds,
                          ZonePosition zone,
                          MecanumDrive drive,
                          PIDFController headingPID,
                          LoopProfiler health,
                          CsvLogger log,
                          ElapsedTime matchTimer,
                          DoubleSupplier forward,
                          DoubleSupplier strafe) {
        ElapsedTime phaseTimer = new ElapsedTime();
        while (opModeIsActive() && phaseTimer.seconds() < durationSeconds) {
            health.tick();

            double headingRad = drive.getHeading();
            double headingDeg = Math.toDegrees(headingRad);
            double turnCorrection = headingPID.calculate(headingDeg);

            drive.driveRobotCentric(forward.get(), strafe.get(), turnCorrection);

            log.writeRow(
                    String.format("%.3f", matchTimer.seconds()),
                    name, zone,
                    String.format("%.1f", headingDeg),
                    String.format("%.1f", health.lastLoopMs()),
                    String.format("%.2f", health.voltage()));

            telemetry.addData("Phase", name);
            telemetry.addData("Zone", zone);
            telemetry.addData("Heading (deg)", "%.1f", headingDeg);
            health.addTelemetry(telemetry);
            telemetry.update();
        }
    }

    @FunctionalInterface
    private interface DoubleSupplier {
        double get();
    }
}
