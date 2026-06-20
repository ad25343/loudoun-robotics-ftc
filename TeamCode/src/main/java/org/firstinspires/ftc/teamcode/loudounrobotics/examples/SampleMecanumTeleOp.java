/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.GamepadEx;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.MecanumDrive;

/**
 * Field-centric mecanum TeleOp that uses three LR helpers together:
 *
 *   • {@link BulkCache}     — set up bulk reads so the loop runs fast
 *   • {@link MecanumDrive}  — robot-centric / field-centric driving
 *   • {@link GamepadEx}     — debounced gamepad for the field-centric toggle
 *
 * Controls (gamepad1):
 *   • Left stick    — translate (forward / backward / strafe)
 *   • Right stick X — turn
 *   • Options       — zero IMU yaw (recalibrate "forward")
 *   • A             — toggle field-centric ↔ robot-centric driving
 *
 * Hardware configuration expected on the Driver Station:
 *   • DcMotor "fl"   — front-left  drive
 *   • DcMotor "bl"   — back-left   drive
 *   • DcMotor "fr"   — front-right drive
 *   • DcMotor "br"   — back-right  drive
 *   • IMU      "imu" — REV Hub IMU
 *
 * If your motor names are different, change the strings in the
 * MecanumDrive constructor below — that's the only edit you need.
 */
@TeleOp(name = "LR Sample: Mecanum TeleOp", group = "Loudoun")
public class SampleMecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        BulkCache cache = new BulkCache(hardwareMap);  // MANUAL mode — call clearAll() per loop

        MecanumDrive drive = new MecanumDrive(hardwareMap,
                "fl", "bl", "fr", "br", "imu");

        GamepadEx gp1 = new GamepadEx(gamepad1);
        // Default to robot-centric. Field-centric requires the driver to know which
        // way the robot was facing at init — easy to get wrong on the first push.
        boolean fieldCentric = false;

        telemetry.addLine("Press PLAY. Left stick = drive, right stick = turn.");
        telemetry.addLine("Press A to toggle field-centric mode (currently OFF — robot-centric).");
        telemetry.addLine("Press OPTIONS to zero heading (only matters in field-centric).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            cache.clearAll();   // refresh bulk-read cache at top of every loop
            gp1.update();

            if (gp1.wasJustPressed(g -> g.a)) {
                fieldCentric = !fieldCentric;
            }
            if (gp1.wasJustPressed(g -> g.options)) {
                drive.resetHeading();
            }

            double forward = -gamepad1.left_stick_y;   // stick Y is inverted in hardware
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            if (fieldCentric) {
                drive.driveFieldCentric(forward, strafe, turn);
            } else {
                drive.driveRobotCentric(forward, strafe, turn);
            }

            telemetry.addData("Drive mode", fieldCentric ? "field-centric" : "robot-centric");
            telemetry.addData("Heading (deg)", Math.toDegrees(drive.getHeading()));
            telemetry.update();
        }

        drive.stop();
    }
}
