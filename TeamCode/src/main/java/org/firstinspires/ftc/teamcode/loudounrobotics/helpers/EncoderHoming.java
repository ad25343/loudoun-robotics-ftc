/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Generic encoder-homing routine: drive a motor at low power until a limit
 * switch trips, then zero the encoder. Standard pattern for any team with
 * a linear slide, arm, or lift that needs a known starting position.
 *
 * Why home? Encoders count relative position, not absolute. After power-on
 * or a mid-match reset, the encoder thinks "zero" is wherever the mechanism
 * happens to be. Homing fixes that — drive the mechanism to its physical
 * stop (where the limit switch is), then call that position zero.
 *
 * Usage (Autonomous or init phase of TeleOp):
 * <pre>
 *   boolean homed = EncoderHoming.runUntilSwitch(
 *       slideMotor,
 *       slideLimitSwitch,
 *       -0.3,    // homing power, signed. negative = retract direction here.
 *       5.0,     // timeout in seconds — fail-safe if switch never trips
 *       this     // the LinearOpMode (used to honor opModeIsActive())
 *   );
 *
 *   if (!homed) {
 *       telemetry.addLine("WARN: slide didn't home — limit switch fault?");
 *   }
 * </pre>
 *
 * Sign convention: pass {@code power} as negative if "home" is the retracted
 * direction (most common). Pass positive if your mechanism homes by extending.
 * The method does NOT change the sign for you — it pushes whatever you give it.
 */
public final class EncoderHoming {

    private EncoderHoming() { /* utility class */ }

    /**
     * @param motor          the motor to drive
     * @param limitSwitch    the limit switch that defines "home"
     * @param power          signed drive power (typically -0.2 to -0.5 for retraction)
     * @param timeoutSeconds maximum time to attempt homing before giving up
     * @param opMode         the calling LinearOpMode (used for opModeIsActive() safety)
     * @return true if the switch tripped within the timeout, false otherwise
     */
    public static boolean runUntilSwitch(DcMotor motor,
                                         TouchSensor limitSwitch,
                                         double power,
                                         double timeoutSeconds,
                                         LinearOpMode opMode) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);

        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive()
                && !limitSwitch.isPressed()
                && timer.seconds() < timeoutSeconds) {
            // wait — limit switch will end the loop, or the timeout will
        }

        motor.setPower(0);
        boolean success = limitSwitch.isPressed();

        if (success) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return success;
    }
}
