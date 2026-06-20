/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

/**
 * Plain-Java PIDF controller — proportional, integral, derivative, feedforward.
 *
 * Why a custom controller when the SDK has {@code PIDFCoefficients}? The SDK's
 * built-in is wired into specific motor modes and is hard to use for anything
 * else (vision-driven turn alignment, drive distance, mechanism position, etc.).
 * This one operates on any {@code double} measurement.
 *
 * The math:
 * <pre>
 *   error = setpoint - measurement
 *   integral += error * dt
 *   derivative = (error - lastError) / dt
 *   output = kP*error + kI*integral + kD*derivative + kF*setpoint
 * </pre>
 *
 * Output is clamped to {@code [-1.0, 1.0]} by default — the motor power range.
 * Override with {@link #setOutputLimits(double, double)}.
 *
 * Integral windup is mitigated by an absolute cap (default Infinity = off) —
 * set via {@link #setIntegralCap(double)} for systems that can integrate
 * faster than they can respond (gravity-loaded arms, etc.).
 *
 * Tuning, in order:
 *   1. Set kI = kD = kF = 0.
 *   2. Raise kP until the system oscillates. Halve it.
 *   3. Add kD until oscillation damps out.
 *   4. Add small kI if there's steady-state error you can't kill with kP.
 *   5. kF is constant pre-bias (e.g., gravity on an arm) — usually leave 0
 *      unless you know you need it.
 *
 * Usage — drive a slide to a target encoder position:
 * <pre>
 *   PIDFController slidePID = new PIDFController(0.005, 0, 0.0001, 0);
 *   slidePID.setSetpoint(2500);
 *
 *   while (opModeIsActive() && !slidePID.atSetpoint(20)) {
 *       double power = slidePID.calculate(slideMotor.getCurrentPosition());
 *       slideMotor.setPower(power);
 *   }
 *   slideMotor.setPower(0);
 * </pre>
 *
 * Usage — turn the robot to face a target heading (degrees):
 * <pre>
 *   PIDFController turnPID = new PIDFController(0.02, 0, 0.003, 0);
 *   turnPID.setSetpoint(targetHeadingDeg);
 *   while (opModeIsActive() && !turnPID.atSetpoint(2.0)) {
 *       double currentHeading = Math.toDegrees(drive.getHeading());
 *       drive.driveRobotCentric(0, 0, turnPID.calculate(currentHeading));
 *   }
 * </pre>
 */
public class PIDFController {

    private double kP, kI, kD, kF;
    private double kS = 0;                  // constant feedforward (gravity / stiction)
    private double setpoint = 0;

    private double lastError = 0;
    private double integral = 0;
    private long lastTimeNs = -1;

    private double outputMin = -1.0;
    private double outputMax = 1.0;
    // Default cap = 1.0 to mitigate integral windup. Tune via setIntegralCap()
    // if you actually need more authority from the I term.
    private double integralCap = 1.0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Convenience for PID-only controllers (no feedforward). */
    public PIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF;
    }

    public void setSetpoint(double setpoint) { this.setpoint = setpoint; }
    public double getSetpoint()              { return setpoint; }

    /** Clamp the output. Defaults to [-1.0, 1.0] (motor power range). */
    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    /** Bound the integral term's absolute value. Default 1.0 — prevents the classic windup trap. */
    public void setIntegralCap(double cap) {
        this.integralCap = Math.abs(cap);
    }

    /**
     * Constant feedforward term — added to the output every cycle, sign-matched to the error.
     * Use for gravity / stiction compensation (e.g., an arm always needs a fixed bias to hold against gravity).
     *
     * NOTE: {@code kF} (the parameter in the constructor) is VELOCITY feedforward —
     * scales with the setpoint magnitude. {@code kS} (this) is CONSTANT — same magnitude
     * every loop. Most teams want kS, not kF, for arms and slides.
     */
    public void setStaticFeedforward(double kS) {
        this.kS = kS;
    }

    /** Clear integral + derivative state — call when the setpoint changes drastically. */
    public void reset() {
        integral = 0;
        lastError = 0;
        lastTimeNs = -1;
    }

    /**
     * Compute the next control output.
     * @param measurement the current process variable
     * @return clamped output, suitable for {@code motor.setPower()}
     */
    public double calculate(double measurement) {
        double error = setpoint - measurement;
        long nowNs = System.nanoTime();
        boolean firstCall = (lastTimeNs < 0);
        double dt = firstCall ? 0.02 : (nowNs - lastTimeNs) / 1e9;  // 20ms default on first call
        lastTimeNs = nowNs;

        // Skip integral on the first call — there's no real dt yet, so its contribution
        // would be a stale ~20ms × current error.
        if (!firstCall) {
            integral += error * dt;
            integral = Math.max(-integralCap, Math.min(integralCap, integral));
        }

        // Skip derivative on the first call — lastError is 0, so it'd be a huge kick.
        double derivative = firstCall ? 0 : (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative + kF * setpoint
                      + kS * Math.signum(error);
        return Math.max(outputMin, Math.min(outputMax, output));
    }

    /** Most recent error (setpoint − measurement). Zero until first calculate() call. */
    public double getError() {
        return lastError;
    }

    /** True if the most recent error is within the given absolute tolerance. */
    public boolean atSetpoint(double tolerance) {
        return Math.abs(lastError) <= tolerance;
    }
}
