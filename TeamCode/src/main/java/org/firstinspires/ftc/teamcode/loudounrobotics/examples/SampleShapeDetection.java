/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.loudounrobotics.helpers.ColorShape;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * TeleOp that streams what {@link ColorShape} is seeing in real time. Use it
 * to figure out two things:
 *
 *   1. What shapes the detector picks up under your camera + lighting
 *   2. Where to set thresholds so noise drops away but the real target
 *      still registers
 *
 * Controls:
 *   • Press PLAY to start
 *   • The driver-station preview shows your camera with green dots at
 *     each detected shape's center, labeled by shape name
 *   • Telemetry shows the best detection (largest) and how many total
 *     detections this frame
 *
 * Tuning workflow:
 *   1. Hold a red object in front of the camera. You should see GREEN
 *      dots tracking it
 *   2. If you see junk detections on table edges, raise the pixel
 *      threshold (call {@code detector.setPixelThreshold(170)}) or the
 *      minimum contour area ({@code detector.setMinContourArea(2500)})
 *   3. If your real object is missed, lower the pixel threshold or
 *      check the lighting — YCrCb is robust but extreme glare can wash
 *      Cr/Cb values
 *
 * Hardware required:
 *   • WebcamName "Webcam 1" — any USB webcam
 */
@TeleOp(name = "LR Sample: Shape Detection", group = "Loudoun")
public class SampleShapeDetection extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Detect any red shape. To detect blue: ColorShape.forBlueShapes()
        // To filter to one shape only: .expect(ColorShape.Shape.TRIANGLE)
        ColorShape detector = ColorShape.forRedShapes();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(detector)
                .build();

        telemetry.addLine("Press PLAY. Then point the camera at colored objects.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<ColorShape.Detection> all = detector.getDetections();
            ColorShape.Detection best = detector.getBestDetection();

            telemetry.addData("Detections", all.size());
            if (best != null) {
                telemetry.addData("Best shape",   best.shape);
                telemetry.addData("Best center",  "(%.0f, %.0f)", best.center.x, best.center.y);
                telemetry.addData("Best area",    "%.0f", best.area);
                telemetry.addData("Vertex count", best.vertexCount);
                telemetry.addData("Circularity",  "%.2f", best.circularity);
            } else {
                telemetry.addLine("No shapes detected — try a brighter / closer object,");
                telemetry.addLine("or lower the pixel threshold in setup.");
            }
            telemetry.update();
        }

        portal.close();
    }
}
