/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Debounced wrapper around the FTC SDK Gamepad class.
 *
 * The raw Gamepad object reports the CURRENT state of every button every loop.
 * If you write {@code if (gamepad1.a) {...}}, the block runs on every loop the
 * driver holds the button — usually not what you want.
 *
 * GamepadEx tracks the previous loop's state so you can detect transitions:
 *   - wasJustPressed  — fires once on the rising edge (press)
 *   - wasJustReleased — fires once on the falling edge (release)
 *   - isPressed       — same as raw Gamepad (current state)
 *
 * Usage:
 * <pre>
 *   GamepadEx gp1 = new GamepadEx(gamepad1);
 *
 *   while (opModeIsActive()) {
 *       gp1.update();   // call once per loop, BEFORE checking buttons
 *
 *       if (gp1.wasJustPressed(g -> g.a)) {
 *           // fires exactly once when A is pressed
 *       }
 *
 *       if (gp1.isPressed(g -> g.right_bumper)) {
 *           // continuous — runs every loop while held
 *       }
 *   }
 * </pre>
 */
public class GamepadEx {

    /** Reads a boolean field from a Gamepad (e.g. {@code g -> g.a}). */
    @FunctionalInterface
    public interface ButtonRef {
        boolean get(Gamepad g);
    }

    private final Gamepad live;
    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    public GamepadEx(Gamepad live) {
        this.live = live;
    }

    /** Call once per loop iteration, before any button checks. */
    public void update() {
        previous.copy(current);
        current.copy(live);
    }

    /** True for ONE loop on the loop where the button transitioned from up to down. */
    public boolean wasJustPressed(ButtonRef button) {
        return button.get(current) && !button.get(previous);
    }

    /** True for ONE loop on the loop where the button transitioned from down to up. */
    public boolean wasJustReleased(ButtonRef button) {
        return !button.get(current) && button.get(previous);
    }

    /** True every loop the button is currently held. */
    public boolean isPressed(ButtonRef button) {
        return button.get(current);
    }
}
