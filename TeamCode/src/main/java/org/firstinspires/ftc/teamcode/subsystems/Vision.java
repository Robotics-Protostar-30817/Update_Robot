package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Vision subsystem tuned for the goBILDA Global Shutter USB Camera (3122-0004-0001).
 * Native sensor aspect ratio is 16:10 (1280x800 or 640x400).
 */
public class Vision implements Subsystem {

    public static final Vision INSTANCE = new Vision();

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Telemetry telemetry;

    // -------------------------------------------------------------------------
    // 1. CAMERA RESOLUTION & LENS INTRINSICS (goBILDA 3122-0004-0001 / OV9281)
    // -------------------------------------------------------------------------
    // Use 640x400 for fastest loop/FPS during Auto, or 1280x800 for long-range tags.
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 400;

    // Calibrated intrinsics for 640x400 (scale x2 if switching to 1280x800).
    public static double LENS_FX = 415.0;
    public static double LENS_FY = 415.0;
    public static final double LENS_CX = CAMERA_WIDTH / 2.0;
    public static final double LENS_CY = CAMERA_HEIGHT / 2.0;

    // -------------------------------------------------------------------------
    // 2. PHYSICAL MOUNTING OFFSETS (Calibrate using CameraCalibrationOpMode)
    // -------------------------------------------------------------------------
    // FTC Robot Coordinate Frame: +X = Right, +Y = Forward, +Z = Up (from robot/turret center)[cite: 7]
    public static double CAMERA_TO_CENTER_X_INCHES = -3.5;
    public static double CAMERA_TO_CENTER_Y_INCHES = -4.25;
    public static double CAMERA_TO_CENTER_Z_INCHES = 12.0;

    // Facing straight forward on the new mount (0.0 deg default; tune with CameraCalibrationOpMode)
    public static double CAMERA_UPWARD_TILT_DEGREES = 27.0;
    public static double CAMERA_YAW_DEGREES = 0.0;
    public static double CAMERA_ROLL_DEGREES = 0.0;

    // Cached frame data updated once per periodic() call
    private List<AprilTagDetection> cachedDetections = Collections.emptyList();
    private Cell redAudienceCell = null;
    private Cell blueAudienceCell = null;
    private boolean enableTelemetry = false;

    public static class Cell {
        public enum Color { RED, BLUE, UNKNOWN }
        public enum Location { AUDIENCE, SCORING, UNKNOWN }

        private final String name;
        private final boolean scorable;
        private final double range;
        private final double bearing;
        private final double elevation;
        private final double roll;
        private final Color color;
        private final Location location;

        // Robot-center-relative coordinates in inches (+X Right, +Y Forward, +Z Up)[cite: 7]
        private final double x;
        private final double y;
        private final double z;

        public Cell(String name,
                    double robotX, double robotY, double robotZ,
                    boolean scorable, double range, double bearing, double elevation, double roll) {
            this.name = name;
            this.x = robotX;
            this.y = robotY;
            this.z = robotZ;
            this.scorable = scorable;
            this.range = range;
            this.bearing = bearing;
            this.elevation = elevation;
            this.roll = roll;

            String upperName = (name == null) ? "" : name.toUpperCase(Locale.ROOT);
            if (upperName.contains("RED")) {
                color = Color.RED;
            } else if (upperName.contains("BLUE")) {
                color = Color.BLUE;
            } else {
                color = Color.UNKNOWN;
            }

            if (upperName.contains("AUDIENCE")) {
                location = Location.AUDIENCE;
            } else if (upperName.contains("SCORING")) {
                location = Location.SCORING;
            } else {
                location = Location.UNKNOWN;
            }
        }

        public String getName() { return name; }
        public double getX() { return x; }
        public double getY() { return y; }
        public double getZ() { return z; }
        public boolean isScorable() { return scorable; }
        public double getRange() { return range; }
        public double getBearing() { return bearing; }
        public double getElevation() { return elevation; }
        public double getRoll() { return roll; }
        public Color getColor() { return color; }
        public Location getLocation() { return location; }
        public boolean isAudience() { return location == Location.AUDIENCE; }
        public boolean isScoring() { return location == Location.SCORING; }
        public boolean isRed() { return color == Color.RED; }
        public boolean isBlue() { return color == Color.BLUE; }
    }

    private Vision() {}

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        rebuildPortal(hardwareMap);
    }

    /**
     * Builds or rebuilds the VisionPortal with current camera mounting offsets and intrinsics.
     * Can be called from CameraCalibrationOpMode after live-tuning offsets.
     */
    public void rebuildPortal(HardwareMap hardwareMap) {
        if (visionPortal != null) {
            visionPortal.close();
        }

        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                CAMERA_TO_CENTER_X_INCHES,
                CAMERA_TO_CENTER_Y_INCHES,
                CAMERA_TO_CENTER_Z_INCHES,
                0
        );

        // In FTC SDK, Pitch = -90 is level forward; adding upward tilt pitches the lens up[cite: 7]
        YawPitchRollAngles cameraAngles = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                CAMERA_YAW_DEGREES,
                -90.0 + CAMERA_UPWARD_TILT_DEGREES,
                CAMERA_ROLL_DEGREES,
                0
        );

        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(LENS_FX, LENS_FY, LENS_CX, LENS_CY)
                .setCameraPose(cameraPosition, cameraAngles)
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam1");
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .build();
    }

    public void setEnableTelemetry(boolean enable) {
        this.enableTelemetry = enable;
    }

    @Override
    public void periodic() {
        if (aprilTag == null) return;

        cachedDetections = aprilTag.getDetections();
        redAudienceCell = findBestCell(Cell.Color.RED, Cell.Location.AUDIENCE);
        blueAudienceCell = findBestCell(Cell.Color.BLUE, Cell.Location.AUDIENCE);

        if (enableTelemetry && telemetry != null) {
            telemetry.addData("AprilTags Detected", cachedDetections.size());
            if (redAudienceCell != null) {
                telemetry.addData("Red Audience Cell",
                        "RobotXY=(%.1f, %.1f) Z=%.1f Range=%.1f Scorable=%b",
                        redAudienceCell.getX(), redAudienceCell.getY(), redAudienceCell.getZ(),
                        redAudienceCell.getRange(), redAudienceCell.isScorable());
            }
            // Note: Do NOT call telemetry.update() inside Subsystem.periodic()
        }
    }

    /**
     * Converts camera-relative ftcPose into robot-center-relative [x, y, z] coordinates (inches).
     * Frame convention: +X = Right, +Y = Forward, +Z = Up from robot/turret center[cite: 7].
     */
    public static double[] cameraToRobotCenter(AprilTagPoseFtc ftcPose) {
        double camX = ftcPose.x;
        double camY = ftcPose.y;
        double camZ = ftcPose.z;

        double pitch = Math.toRadians(CAMERA_UPWARD_TILT_DEGREES);
        double yaw = Math.toRadians(CAMERA_YAW_DEGREES);

        // 1. Un-tilt camera upward pitch to get floor-parallel coordinates[cite: 7]
        double levelX = camX;
        double levelY = camY * Math.cos(pitch) - camZ * Math.sin(pitch);
        double levelZ = camY * Math.sin(pitch) + camZ * Math.cos(pitch);

        // 2. Rotate by FTC counter-clockwise yaw (+ = camera aimed left, - = camera aimed right)
        double rx = (levelX * Math.cos(yaw) - levelY * Math.sin(yaw)) + CAMERA_TO_CENTER_X_INCHES;
        double ry = (levelX * Math.sin(yaw) + levelY * Math.cos(yaw)) + CAMERA_TO_CENTER_Y_INCHES;
        double rz = levelZ + CAMERA_TO_CENTER_Z_INCHES;

        return new double[]{rx, ry, rz};
    }

    private Cell findBestCell(Cell.Color targetColor, Cell.Location targetLocation) {
        AprilTagClusterDetection best = null;
        double bestPercent = -1.0;

        for (AprilTagDetection detection : cachedDetections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

            if (cluster.metadata == null || cluster.metadata.name == null || cluster.ftcPose == null) {
                continue;
            }
            if (!isCellScorable(cluster)) continue;

            String upper = cluster.metadata.name.toUpperCase(Locale.ROOT);
            boolean colorMatch = (targetColor == Cell.Color.RED && upper.contains("RED"))
                    || (targetColor == Cell.Color.BLUE && upper.contains("BLUE"));
            boolean locMatch = (targetLocation == Cell.Location.AUDIENCE && upper.contains("AUDIENCE"))
                    || (targetLocation == Cell.Location.SCORING && upper.contains("SCORING"));

            if (colorMatch && locMatch && cluster.percentClusterFound > bestPercent) {
                bestPercent = cluster.percentClusterFound;
                best = cluster;
            }
        }

        if (best == null) return null;

        // Transform camera-relative ftcPose into robot-center-relative coordinates
        double[] robotCoords = cameraToRobotCenter(best.ftcPose);

        return new Cell(
                best.metadata.name,
                robotCoords[0], robotCoords[1], robotCoords[2],
                true,
                best.ftcPose.range,
                best.ftcPose.bearing,
                best.ftcPose.elevation,
                best.ftcPose.roll
        );
    }

    public boolean isRedAudienceScorable() {
        return redAudienceCell != null && redAudienceCell.isScorable();
    }

    public boolean isBlueAudienceScorable() {
        return blueAudienceCell != null && blueAudienceCell.isScorable();
    }

    public Cell getRedAudienceCell() {
        return redAudienceCell;
    }

    public Cell getBlueAudienceCell() {
        return blueAudienceCell;
    }

    public Cell getBestCell(Cell.Color color) {
        if (color == Cell.Color.RED) return redAudienceCell;
        if (color == Cell.Color.BLUE) return blueAudienceCell;
        return null;
    }

    public List<AprilTagDetection> getDetections() {
        return cachedDetections;
    }

    public boolean isCellScorable(AprilTagClusterDetection cluster) {
        if (cluster == null || cluster.ftcPose == null) return false;
        return Math.abs(cluster.ftcPose.roll) < 90.0;
    }

    /** NextFTC command to pause camera streaming once scoring is complete */
    public Command pauseStream() {
        return new InstantCommand(() -> {
            if (visionPortal != null) {
                visionPortal.stopStreaming();
            }
        });
    }

    public VisionPortal getVisionPortal() { return visionPortal; }
    public AprilTagProcessor getAprilTagProcessor() { return aprilTag; }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}