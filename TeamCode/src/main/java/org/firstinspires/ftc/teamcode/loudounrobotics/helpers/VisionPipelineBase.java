/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Abstract base for shape-detection vision pipelines.
 *
 * Implements the canonical FTC computer-vision preprocessing:
 *   1. Gaussian blur (denoise)
 *   2. RGB → grayscale
 *   3. Canny edge detection
 *   4. Dilate (thicken edges to connect broken contours)
 *   5. findContours
 *
 * Subclasses override {@link #analyze(Mat, List)} to make decisions
 * from the resulting contours — e.g., "find the largest rectangle,"
 * "count contours with 3 vertices," "locate the centroid of the
 * biggest blob."
 *
 * Pair this with {@link ThreeZoneDetector} for color-based detection.
 * Use this for shape-based detection (corners, contour areas, etc.).
 *
 * Tunable preprocessing parameters:
 *   • {@link #setBlurSize(int)}      — Gaussian kernel size (default 7)
 *   • {@link #setCannyThresholds(double, double)} — edge sensitivity (default 200/25)
 *   • {@link #setDilateIterations(int)} — edge thickening passes (default 1)
 *
 * Usage — find the largest contour in the frame:
 * <pre>
 *   public class LargestContourFinder extends VisionPipelineBase {
 *       private volatile double largestArea = 0;
 *
 *       &#64;Override
 *       protected Object analyze(Mat frame, List&lt;MatOfPoint&gt; contours) {
 *           double max = 0;
 *           for (MatOfPoint c : contours) {
 *               double area = Imgproc.contourArea(c);
 *               if (area &gt; max) max = area;
 *           }
 *           largestArea = max;
 *           return max;
 *       }
 *
 *       public double getLargestArea() { return largestArea; }
 *   }
 * </pre>
 */
public abstract class VisionPipelineBase implements VisionProcessor {

    // Reused Mats to avoid per-frame allocation (avoids memory creep).
    private final Mat blurred = new Mat();
    private final Mat gray    = new Mat();
    private final Mat edges   = new Mat();
    private final Mat hierarchy = new Mat();

    private int blurSize = 7;
    private double cannyLow = 25;
    private double cannyHigh = 200;
    private int dilateIterations = 1;
    private double minContourArea = 1000;

    public void setBlurSize(int blurSize)            { this.blurSize = blurSize; }
    public void setCannyThresholds(double low, double high) { this.cannyLow = low; this.cannyHigh = high; }
    public void setDilateIterations(int iterations)  { this.dilateIterations = iterations; }
    /** Contours below this area are filtered out as noise. Default 1000. */
    public void setMinContourArea(double area)       { this.minContourArea = area; }

    @Override
    public final Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.GaussianBlur(frame, blurred, new Size(blurSize, blurSize), 1);
        Imgproc.cvtColor(blurred, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(gray, edges, cannyLow, cannyHigh);
        Imgproc.dilate(edges, edges, new Mat(), new Point(-1, -1), dilateIterations);

        List<MatOfPoint> allContours = new ArrayList<>();
        Imgproc.findContours(edges, allContours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        List<MatOfPoint> filtered = new ArrayList<>();
        for (MatOfPoint c : allContours) {
            if (Imgproc.contourArea(c) >= minContourArea) {
                filtered.add(c);
            }
        }

        return analyze(frame, filtered);
    }

    /**
     * Implement your detection logic here.
     *
     * @param frame    the original RGB frame (modify to draw overlays — but prefer
     *                 {@link #onDrawFrame} for that)
     * @param contours contours above the {@link #setMinContourArea(double)} threshold,
     *                 sorted in the order findContours returned them
     * @return any object — VisionProcessor convention says this is passed back to
     *         onDrawFrame as userContext, useful for drawing what was detected
     */
    protected abstract Object analyze(Mat frame, List<MatOfPoint> contours);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // default no-op — override if you need frame dimensions
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // default no-op — override to draw overlays on the camera preview
    }
}
