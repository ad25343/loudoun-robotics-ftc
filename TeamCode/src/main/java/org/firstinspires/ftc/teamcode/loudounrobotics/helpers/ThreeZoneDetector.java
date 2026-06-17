/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Detect which of three horizontal zones (LEFT / MIDDLE / RIGHT) has the
 * most pixels matching a target color. The classic "where is the team
 * element / game prop?" autonomous detector — used every FTC season.
 *
 * <b>Lighting note:</b> this detector uses HSV thresholding, which is
 * sensitive to brightness changes between your practice space and the
 * competition venue. For more lighting-robust detection of red or blue
 * elements, prefer {@link ChrominanceZoneDetector} (YCrCb-based).
 * Keep this class for yellow (yellow doesn't separate cleanly in YCrCb)
 * or when you've already tuned HSV bounds you trust.
 *
 * Plug into a {@code VisionPortal.Builder()} as a processor; read
 * {@link #getDetection()} from your OpMode to find out which zone wins.
 *
 * Pass an HSV color range to the constructor — anything inside the range
 * counts as a "match." For convenience, use one of the static factories:
 *   • {@link #forRedElement()}    — saturated red
 *   • {@link #forBlueElement()}   — saturated blue
 *   • {@link #forYellowElement()} — saturated yellow
 *
 * Or define your own range with {@link Scalar} — H ∈ [0, 180],
 * S ∈ [0, 255], V ∈ [0, 255] (OpenCV's HSV scaling).
 *
 * Calibration:
 *   • If detection bounces between zones, raise {@link #setThreshold(int)}.
 *   • If detection always returns NONE, lower the threshold or widen the
 *     HSV range.
 *   • Print {@link #getZoneSums()} from your OpMode while the prop is in
 *     each zone to see what raw numbers your robot actually sees.
 *
 * Usage:
 * <pre>
 *   ThreeZoneDetector detector = ThreeZoneDetector.forRedElement();
 *
 *   VisionPortal portal = new VisionPortal.Builder()
 *       .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
 *       .addProcessor(detector)
 *       .build();
 *
 *   waitForStart();
 *
 *   ZonePosition where = detector.getDetection();
 *   switch (where) {
 *       case LEFT:   // ... drive autonomous path A
 *       case MIDDLE: // ... drive autonomous path B
 *       case RIGHT:  // ... drive autonomous path C
 *       case NONE:   // ... fallback
 *   }
 * </pre>
 */
public class ThreeZoneDetector implements VisionProcessor {

    public enum ZonePosition { LEFT, MIDDLE, RIGHT, NONE }

    private final Scalar hsvLower;
    private final Scalar hsvUpper;
    private int threshold = 25500;  // ~100 matching pixels minimum (255 per pixel × 100)

    private Rect leftZone, middleZone, rightZone;

    // Reused per-frame so we don't allocate Mats inside processFrame.
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();

    private volatile ZonePosition detection = ZonePosition.NONE;
    private volatile double leftSum = 0, middleSum = 0, rightSum = 0;

    public ThreeZoneDetector(Scalar hsvLowerBound, Scalar hsvUpperBound) {
        this.hsvLower = hsvLowerBound;
        this.hsvUpper = hsvUpperBound;
    }

    public static ThreeZoneDetector forRedElement() {
        return new ThreeZoneDetector(new Scalar(0, 120, 100), new Scalar(15, 255, 255));
    }

    public static ThreeZoneDetector forBlueElement() {
        return new ThreeZoneDetector(new Scalar(100, 120, 100), new Scalar(130, 255, 255));
    }

    public static ThreeZoneDetector forYellowElement() {
        return new ThreeZoneDetector(new Scalar(20, 120, 100), new Scalar(35, 255, 255));
    }

    /** Minimum pixel-sum a zone needs to be picked as the detection. Higher = stricter. */
    public void setThreshold(int threshold) {
        this.threshold = threshold;
    }

    /** Latest detection — thread-safe, can be read from the OpMode loop. */
    public ZonePosition getDetection() {
        return detection;
    }

    /** Raw pixel-sums per zone (left, middle, right). Useful for calibrating the threshold. */
    public double[] getZoneSums() {
        return new double[] { leftSum, middleSum, rightSum };
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        int third = width / 3;
        leftZone   = new Rect(0,             0, third,         height);
        middleZone = new Rect(third,         0, third * 2,     height);
        rightZone  = new Rect(third * 2,     0, width,         height);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, hsvLower, hsvUpper, mask);

        org.opencv.core.Rect leftRect   = toCvRect(leftZone);
        org.opencv.core.Rect middleRect = toCvRect(middleZone);
        org.opencv.core.Rect rightRect  = toCvRect(rightZone);

        leftSum   = Core.sumElems(mask.submat(leftRect)).val[0];
        middleSum = Core.sumElems(mask.submat(middleRect)).val[0];
        rightSum  = Core.sumElems(mask.submat(rightRect)).val[0];

        double max = Math.max(leftSum, Math.max(middleSum, rightSum));
        if (max < threshold) {
            detection = ZonePosition.NONE;
        } else if (max == leftSum) {
            detection = ZonePosition.LEFT;
        } else if (max == middleSum) {
            detection = ZonePosition.MIDDLE;
        } else {
            detection = ZonePosition.RIGHT;
        }

        return detection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        if (canvas == null || leftZone == null) return;

        Paint zoneBorder = new Paint();
        zoneBorder.setStyle(Paint.Style.STROKE);
        zoneBorder.setStrokeWidth(4 * scaleCanvasDensity);

        drawZone(canvas, leftZone,   detection == ZonePosition.LEFT   ? Color.GREEN : Color.WHITE, zoneBorder, scaleBmpPxToCanvasPx);
        drawZone(canvas, middleZone, detection == ZonePosition.MIDDLE ? Color.GREEN : Color.WHITE, zoneBorder, scaleBmpPxToCanvasPx);
        drawZone(canvas, rightZone,  detection == ZonePosition.RIGHT  ? Color.GREEN : Color.WHITE, zoneBorder, scaleBmpPxToCanvasPx);
    }

    private void drawZone(Canvas canvas, Rect zone, int color, Paint paint, float scale) {
        paint.setColor(color);
        canvas.drawRect(new RectF(
                zone.left * scale, zone.top * scale,
                zone.right * scale, zone.bottom * scale), paint);
    }

    private static org.opencv.core.Rect toCvRect(Rect r) {
        return new org.opencv.core.Rect(r.left, r.top, r.width(), r.height());
    }
}
