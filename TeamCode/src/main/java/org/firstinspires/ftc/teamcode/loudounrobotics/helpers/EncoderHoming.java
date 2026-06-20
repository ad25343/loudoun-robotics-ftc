/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
 *
 * Switch wiring: two overloads.
 *   • {@code TouchSensor} — easy: the SDK class normalizes for you. {@code isPressed()}
 *     returns true when the switch is closed regardless of how it's wired.
 *   • {@code DigitalChannel} — raw: returns the actual line state. A normally-open
 *     switch connected to a digital port reads LOW when pressed (the default).
 *     Pass {@code pressedReadsHigh=true} if your switch is normally-closed.
 */
public final class EncoderHoming {

    private EncoderHoming() { /* utility class */ }

    /** TouchSensor variant — convenient, no wiring assumptions to make. */
    public static boolean runUntilSwitch(DcMotor motor,
                                         TouchSensor limitSwitch,
                                         double power,
                                         double timeoutSeconds,
                                         LinearOpMode opMode) {
        return runUntilPressed(motor, power, timeoutSeconds, opMode,
                limitSwitch::isPressed);
    }

    /**
     * DigitalChannel variant — direct read of the digital line.
     * Default ({@code pressedReadsHigh=false}) matches the most common wiring:
     * a normally-open switch grounded to GND, which reads LOW when pressed.
     */
    public static boolean runUntilSwitch(DcMotor motor,
                                         DigitalChannel limitSwitch,
                                         boolean pressedReadsHigh,
                                         double power,
                                         double timeoutSeconds,
                                         LinearOpMode opMode) {
        return runUntilPressed(motor, power, timeoutSeconds, opMode,
                () -> limitSwitch.getState() == pressedReadsHigh);
    }

    /** Shared loop body — sensor-type-agnostic. */
    private static boolean runUntilPressed(DcMotor motor,
                                           double power,
                                           double timeoutSeconds,
                                           LinearOpMode opMode,
                                           PressedCheck pressed) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);

        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive()
                && !pressed.isPressed()
                && timer.seconds() < timeoutSeconds) {
            // idle() yields to the SDK event loop so Lynx comms keep flowing.
            // Busy-spinning here starved the SDK and could trigger watchdog warnings.
            opMode.idle();
        }

        motor.setPower(0);
        boolean success = pressed.isPressed();

        if (success) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Give the controller a tick to apply the mode change before we ask for
            // RUN_USING_ENCODER. Some firmware reports a stale position otherwise.
            try { Thread.sleep(50); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return success;
    }

    @FunctionalInterface
    private interface PressedCheck {
        boolean isPressed();
    }
}
