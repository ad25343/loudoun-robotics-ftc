/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Color-filtered shape detector. The right way to find shapes in an FTC frame.
 *
 * <h2>Why color-first?</h2>
 *
 * Edge detection on the whole frame picks up everything — table boundaries,
 * gym walls, taped lines, other robots, anything with contrast. Shape
 * classification on that noisy contour pile is hopeless.
 *
 * This detector inverts the pipeline: it color-masks FIRST (keeping only
 * pixels matching the target color), then finds contours only within the
 * mask. Most non-target noise is gone before shape logic ever runs.
 *
 * Full pipeline:
 * <pre>
 *   frame → YCrCb → extract Cr (or Cb)
 *         → threshold        (only strong-color pixels remain)
 *         → morphological close   (clean up speckle and small gaps)
 *         → findContours          (only on the masked region)
 *         → approxPolyDP          (vertex count)
 *         → classify by vertex count + circularity
 * </pre>
 *
 * <h2>Shapes returned</h2>
 *
 * <ul>
 *   <li>{@code TRIANGLE}  — 3 vertices</li>
 *   <li>{@code SQUARE}    — 4 vertices, aspect ratio ≈ 1</li>
 *   <li>{@code RECTANGLE} — 4 vertices, aspect ratio ≠ 1</li>
 *   <li>{@code PENTAGON}  — 5 vertices</li>
 *   <li>{@code HEXAGON}   — 6 vertices</li>
 *   <li>{@code CIRCLE}    — many vertices + circularity &gt; 0.85</li>
 *   <li>{@code OTHER}     — none of the above</li>
 * </ul>
 *
 * <h2>Calibration</h2>
 *
 * Three knobs:
 * <ul>
 *   <li>{@link #setPixelThreshold(int)} — chrominance threshold (default 150).
 *       Higher = stricter color match.</li>
 *   <li>{@link #setMinContourArea(double)} — drop tiny contours as noise
 *       (default 1000 pixels).</li>
 *   <li>{@link #expect(Shape)} — only return detections of one shape.</li>
 * </ul>
 *
 * Usage:
 * <pre>
 *   ColorShape detector = ColorShape.forRedShapes().expect(ColorShape.Shape.TRIANGLE);
 *
 *   VisionPortal portal = new VisionPortal.Builder()
 *       .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
 *       .addProcessor(detector)
 *       .build();
 *
 *   while (opModeIsActive()) {
 *       ColorShape.Detection best = detector.getBestDetection();
 *       if (best != null) {
 *           telemetry.addData("Found", "%s at (%.0f, %.0f)",
 *               best.shape, best.center.x, best.center.y);
 *       }
 *       telemetry.update();
 *   }
 * </pre>
 */
public class ColorShape implements VisionProcessor {

    public enum Shape { TRIANGLE, SQUARE, RECTANGLE, PENTAGON, HEXAGON, CIRCLE, OTHER }

    /** Which YCrCb channel to threshold on. Cr = red, Cb = blue. */
    public enum Channel {
        CR(1), CB(2);
        final int index;
        Channel(int index) { this.index = index; }
    }

    private final Channel channel;
    private int pixelThreshold;
    private double minContourArea = 1000;
    private Shape expected = null;  // null = any shape

    // Reused per frame to avoid allocation churn
    private final Mat ycrcb = new Mat();
    private final Mat channelMat = new Mat();
    private final Mat mask = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    private volatile List<Detection> detections = Collections.emptyList();
    private volatile int frameWidth = 0, frameHeight = 0;

    public ColorShape(Channel channel, int pixelThreshold) {
        this.channel = channel;
        this.pixelThreshold = pixelThreshold;
    }

    public static ColorShape forRedShapes()  { return new ColorShape(Channel.CR, 150); }
    public static ColorShape forBlueShapes() { return new ColorShape(Channel.CB, 150); }

    /** Only report detections of this shape. Pass null to clear the filter. */
    public ColorShape expect(Shape shape) {
        this.expected = shape;
        return this;
    }

    public ColorShape setPixelThreshold(int threshold) {
        this.pixelThreshold = threshold;
        return this;
    }

    public ColorShape setMinContourArea(double area) {
        this.minContourArea = area;
        return this;
    }

    /** All matching detections this frame (largest first). Thread-safe to read from the OpMode loop. */
    public List<Detection> getDetections() {
        return detections;
    }

    /** Largest matching detection this frame, or null if none. */
    public Detection getBestDetection() {
        return detections.isEmpty() ? null : detections.get(0);
    }

    public boolean hasDetection() {
        return !detections.isEmpty();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.frameWidth = width;
        this.frameHeight = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // 1. Color mask (lighting-robust chrominance threshold)
        Imgproc.cvtColor(frame, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcb, channelMat, channel.index);
        Imgproc.threshold(channelMat, mask, pixelThreshold, 255, Imgproc.THRESH_BINARY);

        // 2. Morphological close — fill speckle holes inside the mask
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // 3. Find contours only within the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();

        // 4. Classify each contour
        List<Detection> found = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < minContourArea) {
                contour.release();
                continue;
            }

            MatOfPoint2f c2f = new MatOfPoint2f(contour.toArray());
            double perimeter = Imgproc.arcLength(c2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(c2f, approx, 0.02 * perimeter, true);
            int vertexCount = approx.toArray().length;
            double circularity = (perimeter > 0)
                    ? (4 * Math.PI * area) / (perimeter * perimeter)
                    : 0;

            Shape shape = classify(vertexCount, circularity, contour);

            if (expected == null || shape == expected) {
                Moments moments = Imgproc.moments(contour);
                double cx = moments.m00 > 0 ? moments.m10 / moments.m00 : 0;
                double cy = moments.m00 > 0 ? moments.m01 / moments.m00 : 0;
                found.add(new Detection(shape, new Point(cx, cy), area, vertexCount, circularity));
            }

            c2f.release();
            approx.release();
            contour.release();
        }

        // Sort largest first so getBestDetection() picks the most prominent shape
        found.sort((a, b) -> Double.compare(b.area, a.area));
        detections = found;
        return found;
    }

    private static Shape classify(int vertexCount, double circularity, MatOfPoint contour) {
        // Circle: needs BOTH high circularity (close to a perfect circle) AND many vertices
        // (>12 — `8` was too permissive; noisy hexagons frequently approxPolyDP to 8-10).
        if (circularity > 0.85 && vertexCount > 12) return Shape.CIRCLE;

        switch (vertexCount) {
            case 3: return Shape.TRIANGLE;
            case 4:
                // Aspect is from the AXIS-ALIGNED bounding rect — a square rotated 45°
                // reports as a square (its bounding box IS roughly equal-sided). That's fine
                // for "is this a square?" but tells you nothing about orientation. Use
                // minAreaRect() if you need rotation-aware detection.
                Rect br = Imgproc.boundingRect(contour);
                double aspect = (br.height > 0) ? (double) br.width / br.height : 1.0;
                return (aspect > 0.85 && aspect < 1.15) ? Shape.SQUARE : Shape.RECTANGLE;
            case 5:  return Shape.PENTAGON;
            case 6:  return Shape.HEXAGON;
            default: return Shape.OTHER;
        }
    }

    /**
     * Release the OpenCV native resources held by this detector. Call from your OpMode's
     * {@code stop()} or when you swap to a different VisionProcessor. Safe to call multiple times.
     */
    public void close() {
        if (kernel != null) kernel.release();
        ycrcb.release();
        channelMat.release();
        mask.release();
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        if (canvas == null) return;
        List<Detection> snapshot = detections;
        if (snapshot.isEmpty()) return;

        Paint dot = new Paint();
        dot.setStyle(Paint.Style.FILL);
        dot.setColor(Color.GREEN);

        Paint label = new Paint();
        label.setColor(Color.GREEN);
        label.setTextSize(28 * scaleCanvasDensity);
        label.setFakeBoldText(true);

        for (Detection d : snapshot) {
            float x = (float)(d.center.x * scaleBmpPxToCanvasPx);
            float y = (float)(d.center.y * scaleBmpPxToCanvasPx);
            canvas.drawCircle(x, y, 8 * scaleCanvasDensity, dot);
            canvas.drawText(d.shape.name(), x + 12 * scaleCanvasDensity, y, label);
        }
    }

    /** One detected shape. Immutable. */
    public static final class Detection {
        public final Shape shape;
        public final Point center;
        public final double area;
        public final int vertexCount;
        public final double circularity;

        public Detection(Shape shape, Point center, double area,
                         int vertexCount, double circularity) {
            this.shape = shape;
            this.center = center;
            this.area = area;
            this.vertexCount = vertexCount;
            this.circularity = circularity;
        }
    }
}
