/*
 * Copyright (c) 2026 Loudoun Robotics
 * SPDX-License-Identifier: MIT
 */

package org.firstinspires.ftc.teamcode.loudounrobotics.helpers;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Crash-safe CSV logger for the Control Hub.
 *
 * Writes one row per call. If the file can't be opened or the disk fills,
 * the logger silently disables itself — it will NEVER throw an exception
 * that crashes your OpMode. Match logging that can blow up an OpMode is
 * worse than no logging at all.
 *
 * Files land in {@code /sdcard/FIRST/} by default — the standard FTC
 * scratch location on a Control Hub. After the match, pull them off with
 * {@code adb pull /sdcard/FIRST/} or via the REV Hardware Client.
 *
 * Each logger auto-timestamps its filename so you don't overwrite
 * previous runs. For example, {@code new CsvLogger("loop_times")}
 * produces {@code /sdcard/FIRST/loop_times_20260616_143052.csv}.
 *
 * Usage:
 * <pre>
 *   CsvLogger log = new CsvLogger("drive_telemetry");
 *   log.writeHeader("time_s", "voltage", "loop_ms", "heading_deg");
 *
 *   while (opModeIsActive()) {
 *       log.writeRow(matchTime.seconds(), battery, loopMs, drive.getHeading());
 *   }
 *   log.close();  // optional; OS will close on OpMode shutdown
 * </pre>
 */
public class CsvLogger {

    private static final String DEFAULT_DIR = "/sdcard/FIRST/";
    private static final int FLUSH_EVERY_ROWS = 50;
    private static final long FLUSH_EVERY_NANOS = 1_000_000_000L;  // 1 second

    private BufferedWriter writer;
    private boolean open = false;
    private final String filename;
    private int rowsSinceFlush = 0;
    private long lastFlushNanos = 0;

    /** Logs to /sdcard/FIRST/&lt;baseName&gt;_&lt;timestamp&gt;.csv. */
    public CsvLogger(String baseName) {
        this(DEFAULT_DIR, baseName, true);
    }

    /**
     * @param directory     where to write — must end in "/". Will be created if missing.
     * @param baseName      filename without extension
     * @param addTimestamp  if true, appends "_YYYYMMDD_HHmmss" to the filename
     */
    public CsvLogger(String directory, String baseName, boolean addTimestamp) {
        String stamp = addTimestamp
                ? "_" + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date())
                : "";
        this.filename = directory + baseName + stamp + ".csv";
        try {
            new File(directory).mkdirs();  // no-op if it exists
            // 8 KB buffer — large enough that ~50 rows fit before SD-card hits the disk.
            writer = new BufferedWriter(new FileWriter(filename, true), 8192);
            lastFlushNanos = System.nanoTime();
            open = true;
        } catch (Exception e) {
            // Disk full, no write permission, whatever — silently disable.
            open = false;
        }
    }

    /** Write the header row. Same as writeRow() — just here for readability. */
    public void writeHeader(String... columns) {
        writeRow((Object[]) columns);
    }

    /**
     * Write one row. Values containing commas, quotes, or newlines are
     * automatically RFC-4180-quoted (wrapped in {@code "..."} with internal
     * quotes doubled). Null values become empty cells.
     *
     * Writes go to an 8 KB buffer. Flush happens every {@value #FLUSH_EVERY_ROWS}
     * rows or every {@value #FLUSH_EVERY_NANOS}ns (1 second), whichever comes
     * first — keeps SD-card stalls out of the OpMode hot loop. Call
     * {@link #flush()} or {@link #close()} to force.
     */
    public void writeRow(Object... values) {
        if (!open) return;
        try {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < values.length; i++) {
                if (i > 0) sb.append(',');
                sb.append(escape(values[i]));
            }
            writer.append(sb).append('\n');
            rowsSinceFlush++;

            long nowNanos = System.nanoTime();
            if (rowsSinceFlush >= FLUSH_EVERY_ROWS
                    || (nowNanos - lastFlushNanos) >= FLUSH_EVERY_NANOS) {
                writer.flush();
                rowsSinceFlush = 0;
                lastFlushNanos = nowNanos;
            }
        } catch (IOException e) {
            open = false;
            close();
        }
    }

    /** Force a flush to disk. Useful at phase boundaries or before stop. */
    public void flush() {
        if (!open) return;
        try {
            writer.flush();
            rowsSinceFlush = 0;
            lastFlushNanos = System.nanoTime();
        } catch (IOException e) {
            open = false;
            close();
        }
    }

    private static String escape(Object value) {
        if (value == null) return "";
        String s = String.valueOf(value);
        if (s.indexOf(',') >= 0 || s.indexOf('"') >= 0 || s.indexOf('\n') >= 0) {
            return "\"" + s.replace("\"", "\"\"") + "\"";
        }
        return s;
    }

    /** True if the logger is still writing. False after a failure or after {@link #close()}. */
    public boolean isOpen() {
        return open;
    }

    /** Absolute path of the file being written. */
    public String getFilename() {
        return filename;
    }

    /** Flush and close. Safe to call multiple times. */
    public void close() {
        open = false;
        if (writer != null) {
            try { writer.close(); } catch (IOException ignored) { }
            writer = null;
        }
    }
}
