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
import org.opencv.imgproc.Imgproc;

/**
 * Three-zone detector using YCrCb chrominance (lighting-robust variant of
 * {@link ThreeZoneDetector}). Use this when the lighting at competition
 * doesn't match the lighting where you tuned the detector.
 *
 * <h2>Why YCrCb instead of HSV?</h2>
 *
 * HSV splits color into Hue + Saturation + Value. Value is brightness — and
 * it shifts hard between a fluorescent-lit gym, an LED-lit competition floor,
 * and a sunlit practice space. Your "red prop" range tuned at home may miss
 * the same prop at the venue.
 *
 * YCrCb splits color into:
 * <ul>
 *   <li><b>Y</b>  — brightness only (we ignore this)</li>
 *   <li><b>Cr</b> — red-vs-green chrominance (high = red)</li>
 *   <li><b>Cb</b> — blue-vs-yellow chrominance (high = blue)</li>
 * </ul>
 *
 * To detect red, threshold ONLY on Cr — brightness doesn't matter. Same for
 * blue with Cb. This is single-channel detection, which is both more robust
 * and faster than the 3-channel HSV approach in {@link ThreeZoneDetector}.
 *
 * <h2>Limitations</h2>
 *
 * Yellow does NOT cleanly separate in YCrCb — high Cr-ish values overlap
 * with red. For yellow detection, use the HSV {@link ThreeZoneDetector} or
 * a LAB color space variant. This class only supports red (Cr) and blue (Cb).
 *
 * <h2>Calibration</h2>
 *
 * The default threshold (150) works for typical saturated FTC props in
 * most lighting. To tune:
 * <ol>
 *   <li>Run the OpMode with the prop in each zone</li>
 *   <li>Read {@link #getZoneSums()} — raw pixel-sums per zone</li>
 *   <li>Set the threshold to about half the lowest sum seen for a real
 *       detection: {@link #setMinPixelSum(int)}</li>
 * </ol>
 *
 * Usage — drop-in replacement for {@link ThreeZoneDetector}:
 * <pre>
 *   ChrominanceZoneDetector detector = ChrominanceZoneDetector.forRedElement();
 *
 *   VisionPortal portal = new VisionPortal.Builder()
 *       .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
 *       .addProcessor(detector)
 *       .build();
 *
 *   ThreeZoneDetector.ZonePosition where = detector.getDetection();
 * </pre>
 *
 * Returns the same {@link ThreeZoneDetector.ZonePosition} enum so OpModes
 * can switch between detectors without changing downstream logic.
 */
public class ChrominanceZoneDetector implements VisionProcessor {

    /** Which YCrCb channel to threshold on. */
    public enum Channel {
        /** Channel 1 — red. Detect red game elements. */
        CR(1),
        /** Channel 2 — blue. Detect blue game elements. */
        CB(2);

        final int index;
        Channel(int index) { this.index = index; }
    }

    private final Channel channel;
    private int pixelThreshold;       // 0-255, per-pixel value above which the pixel counts
    // Default is recomputed in init() once the frame size is known (≈5% of a zone's pixels).
    // Until then, a sane fallback for 640×480.
    private int minPixelSum = 25_500;
    private boolean minPixelSumOverridden = false;

    private Rect leftZone, middleZone, rightZone;

    // Reused per-frame to avoid allocation
    private final Mat ycrcb = new Mat();
    private final Mat channelMat = new Mat();
    private final Mat mask = new Mat();

    private volatile ThreeZoneDetector.ZonePosition detection = ThreeZoneDetector.ZonePosition.NONE;
    private volatile double leftSum = 0, middleSum = 0, rightSum = 0;

    public ChrominanceZoneDetector(Channel channel, int pixelThreshold) {
        this.channel = channel;
        this.pixelThreshold = pixelThreshold;
    }

    public static ChrominanceZoneDetector forRedElement() {
        // 170 is meaningfully above the neutral midpoint (~128) and avoids tripping
        // on cream-colored field tape, which a 150 threshold tends to catch.
        return new ChrominanceZoneDetector(Channel.CR, 170);
    }

    public static ChrominanceZoneDetector forBlueElement() {
        return new ChrominanceZoneDetector(Channel.CB, 170);
    }

    /** Per-pixel chrominance threshold (0-255). Higher = stricter color match. */
    public void setPixelThreshold(int threshold) {
        this.pixelThreshold = threshold;
    }

    /** Minimum sum of matching pixels (× 255) needed to declare a zone winner. */
    public void setMinPixelSum(int sum) {
        this.minPixelSum = sum;
        this.minPixelSumOverridden = true;
    }

    public ThreeZoneDetector.ZonePosition getDetection() {
        return detection;
    }

    public double[] getZoneSums() {
        return new double[] { leftSum, middleSum, rightSum };
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        int third = width / 3;
        leftZone   = new Rect(0,             0, third,         height);
        middleZone = new Rect(third,         0, third * 2,     height);
        rightZone  = new Rect(third * 2,     0, width,         height);

        // Auto-scale minPixelSum to ~5% of a single zone's pixel count, unless the team
        // already pinned it. 5% balances "ignore speckle" vs "catch a real prop in view."
        if (!minPixelSumOverridden) {
            int zonePixels = third * height;
            minPixelSum = (int)(zonePixels * 0.05 * 255);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcb, channelMat, channel.index);
        Imgproc.threshold(channelMat, mask, pixelThreshold, 255, Imgproc.THRESH_BINARY);

        org.opencv.core.Rect leftRect   = toCvRect(leftZone);
        org.opencv.core.Rect middleRect = toCvRect(middleZone);
        org.opencv.core.Rect rightRect  = toCvRect(rightZone);

        leftSum   = Core.sumElems(mask.submat(leftRect)).val[0];
        middleSum = Core.sumElems(mask.submat(middleRect)).val[0];
        rightSum  = Core.sumElems(mask.submat(rightRect)).val[0];

        double max = Math.max(leftSum, Math.max(middleSum, rightSum));
        if (max < minPixelSum) {
            detection = ThreeZoneDetector.ZonePosition.NONE;
        } else if (max == leftSum) {
            detection = ThreeZoneDetector.ZonePosition.LEFT;
        } else if (max == middleSum) {
            detection = ThreeZoneDetector.ZonePosition.MIDDLE;
        } else {
            detection = ThreeZoneDetector.ZonePosition.RIGHT;
        }

        return detection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        if (canvas == null || leftZone == null) return;

        Paint border = new Paint();
        border.setStyle(Paint.Style.STROKE);
        border.setStrokeWidth(4 * scaleCanvasDensity);

        ThreeZoneDetector.ZonePosition d = detection;
        drawZone(canvas, leftZone,   d == ThreeZoneDetector.ZonePosition.LEFT   ? Color.GREEN : Color.WHITE, border, scaleBmpPxToCanvasPx);
        drawZone(canvas, middleZone, d == ThreeZoneDetector.ZonePosition.MIDDLE ? Color.GREEN : Color.WHITE, border, scaleBmpPxToCanvasPx);
        drawZone(canvas, rightZone,  d == ThreeZoneDetector.ZonePosition.RIGHT  ? Color.GREEN : Color.WHITE, border, scaleBmpPxToCanvasPx);
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
