/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drive helper — robot-centric AND field-centric driving.
 *
 * Wraps the canonical FTC mecanum math (denominator normalization,
 * IMU yaw rotation) in one class. Pass it joystick values; it pushes
 * power to four motors.
 *
 * Assumes the standard 4-motor mecanum layout: front-left, back-left,
 * front-right, back-right. Left side motors are reversed by default
 * (most common config). Override via {@link #setLeftReversed(boolean)}
 * if your robot is wired the other way.
 *
 * Hub orientation defaults to logo-up / USB-forward — the most common
 * REV Control Hub mounting. Override via the explicit-orientation
 * constructor if yours is mounted differently.
 *
 * Usage:
 * <pre>
 *   MecanumDrive drive = new MecanumDrive(hardwareMap,
 *       "fl", "bl", "fr", "br", "imu");
 *
 *   while (opModeIsActive()) {
 *       drive.driveFieldCentric(
 *           -gamepad1.left_stick_y,  // forward (stick Y is inverted)
 *            gamepad1.left_stick_x,  // strafe
 *            gamepad1.right_stick_x  // turn
 *       );
 *       if (gamepad1.options) drive.resetHeading();
 *   }
 * </pre>
 */
public class MecanumDrive {

    private final DcMotor frontLeft, backLeft, frontRight, backRight;
    private final IMU imu;
    private double strafeCorrection = 1.1;   // counteract imperfect strafing (per-robot; measure yours)
    private double maxOutputScale  = 1.0;    // headroom for voltage compensation (cap final output)

    /** Standard constructor — REV hub mounted logo-up, USB-forward. */
    public MecanumDrive(HardwareMap hwMap,
                        String frontLeftName, String backLeftName,
                        String frontRightName, String backRightName,
                        String imuName) {
        this(hwMap, frontLeftName, backLeftName, frontRightName, backRightName, imuName,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    /** Explicit hub orientation — use when your Control Hub isn't logo-up / USB-forward. */
    public MecanumDrive(HardwareMap hwMap,
                        String frontLeftName, String backLeftName,
                        String frontRightName, String backRightName,
                        String imuName,
                        RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                        RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {
        frontLeft  = hwMap.get(DcMotor.class, frontLeftName);
        backLeft   = hwMap.get(DcMotor.class, backLeftName);
        frontRight = hwMap.get(DcMotor.class, frontRightName);
        backRight  = hwMap.get(DcMotor.class, backRightName);

        setLeftReversed(true);

        imu = hwMap.get(IMU.class, imuName);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
    }

    /** Flip whether the left side or right side motors are reversed. */
    public void setLeftReversed(boolean leftReversed) {
        DcMotorSimple.Direction left  = leftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction right = leftReversed ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        frontLeft.setDirection(left);
        backLeft.setDirection(left);
        frontRight.setDirection(right);
        backRight.setDirection(right);
    }

    /**
     * Tune the strafe correction factor. Default 1.1 is folk wisdom — your robot may need
     * 1.0 (no correction) to ~1.3 depending on wheel slip + chassis. Drive the robot diagonally
     * across a tile and measure the angle; tune until the diagonal is true.
     */
    public void setStrafeCorrection(double factor) {
        this.strafeCorrection = factor;
    }

    /**
     * Cap the maximum motor output (0.0 to 1.0). Default 1.0 = no cap.
     *
     * Set this BELOW 1.0 (e.g. 0.85) when you want headroom for voltage compensation —
     * a fresh battery at 13V × 0.85 = ~11V at the motor; at 11V a 0.85 → 1.0 scale-up
     * yields the same motor output, so the robot feels the same as the battery drops.
     * Above 11V it just feels slightly slower. Without headroom there's no compensation room.
     */
    public void setMaxOutputScale(double scale) {
        this.maxOutputScale = Math.max(0.0, Math.min(1.0, scale));
    }

    /**
     * Robot-centric driving: forward is whichever way the robot is currently facing.
     * @param forward  + = forward, - = backward
     * @param strafe   + = right, - = left
     * @param turn     + = clockwise, - = counter-clockwise
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        applyPowers(forward, strafe, turn);
    }

    /**
     * Field-centric driving: forward is always the direction the robot was facing
     * at the last {@link #resetHeading()} call (or OpMode start).
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeading();
        // Rotate the requested motion vector by the negative of the robot's heading
        double rotForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotStrafe  = forward * Math.sin(-heading) + strafe * Math.cos(-heading);
        applyPowers(rotForward, rotStrafe, turn);
    }

    /** Zero the IMU yaw — call this when the driver wants "forward" to be the current direction. */
    public void resetHeading() {
        imu.resetYaw();
    }

    /** Current heading in radians, range [-pi, pi]. */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Cut all drive power. */
    public void stop() {
        applyPowers(0, 0, 0);
    }

    private void applyPowers(double forward, double strafe, double turn) {
        // Apply strafe correction BEFORE normalization so both centric modes
        // saturate identically. (Mistake: applying it after the denominator gives
        // the field-centric path different feel than robot-centric.)
        strafe *= strafeCorrection;

        // Denominator: largest absolute power, or 1.0. Keeps ratios when any wheel saturates.
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        frontLeft.setPower( ((forward + strafe + turn) / denominator) * maxOutputScale);
        backLeft.setPower(  ((forward - strafe + turn) / denominator) * maxOutputScale);
        frontRight.setPower(((forward - strafe - turn) / denominator) * maxOutputScale);
        backRight.setPower( ((forward + strafe - turn) / denominator) * maxOutputScale);
    }
}
