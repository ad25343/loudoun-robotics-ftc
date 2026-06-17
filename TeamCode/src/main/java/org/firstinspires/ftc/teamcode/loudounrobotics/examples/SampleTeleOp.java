/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.GamepadEx;

/**
 * Minimal TeleOp showing how to use GamepadEx for one-shot button presses.
 *
 * Press A to toggle a "shooter armed" state. Without GamepadEx, holding A for
 * one second would flip the state ~60 times. With GamepadEx, it flips exactly
 * once per press.
 *
 * This OpMode does NOT touch any hardware — it just prints to telemetry.
 * Safe to run on a control hub with nothing attached.
 */
@TeleOp(name = "LR Sample: GamepadEx", group = "Loudoun")
public class SampleTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        GamepadEx gp1 = new GamepadEx(gamepad1);
        boolean shooterArmed = false;
        int pressCount = 0;

        telemetry.addLine("Press PLAY to start. Then press A on Gamepad 1.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gp1.update();

            if (gp1.wasJustPressed(g -> g.a)) {
                shooterArmed = !shooterArmed;
                pressCount++;
            }

            telemetry.addData("Shooter armed", shooterArmed);
            telemetry.addData("A press count", pressCount);
            telemetry.addLine();
            telemetry.addLine("Hold A — count should NOT keep climbing");
            telemetry.addLine("(that's the whole point of GamepadEx)");
            telemetry.update();
        }
    }
}
