package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;
import java.util.Locale;

/**
 * Interactive TeleOp to calibrate and record the goBILDA 3122-0004-0001 camera pose and intrinsics.
 * Filters exclusively for AprilTagClusterDetection targets.
 *
 * Controls (Gamepad 1):
 * - D-Pad Up / Down: Adjust Camera Upward Tilt (+/- 0.5 deg)
 * - D-Pad Left / Right: Manually Adjust Camera Yaw (+/- 0.2 deg)
 * - Button Y: AUTO-SOLVE Camera Yaw (place target cluster directly on robot centerline X=0 first!)
 * - Right / Left Bumper: Adjust Focal Length LENS_FX and LENS_FY (+/- 2.0 px)
 * - Button X: RECORD current calibration constants to screen and RobotLog
 * - Button A: Reset Yaw to 0.0 deg (straight forward) and rebuild portal
 */
@TeleOp(name = "Camera Calibration Test", group = "Calibration")
public class CameraTest extends LinearOpMode {

    private static final String LOG_TAG = "CameraCal";

    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLeft = false;
    private boolean prevRight = false;
    private boolean prevRb = false;
    private boolean prevLb = false;
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevY = false;

    private int recordCount = 0;
    private String lastRecordedSummary = "None yet (Press X to record)";

    @Override
    public void runOpMode() {
        Vision.INSTANCE.init(hardwareMap, telemetry);
        Vision.INSTANCE.setEnableTelemetry(false);

        telemetry.addLine("Camera Calibration Ready (AprilTagCluster Mode).");
        telemetry.addLine("1. Place target cluster dead-center ahead of robot (X = 0).");
        telemetry.addLine("2. Press PLAY, press Y to auto-solve Yaw, and press X to record.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean needsRebuild = false;

            // D-Pad Up / Down: Adjust Camera Upward Tilt (+/- 0.5 deg)
            if (gamepad1.dpad_up && !prevUp) {
                Vision.CAMERA_UPWARD_TILT_DEGREES += 0.5;
                needsRebuild = true;
            }
            if (gamepad1.dpad_down && !prevDown) {
                Vision.CAMERA_UPWARD_TILT_DEGREES -= 0.5;
                needsRebuild = true;
            }

            // D-Pad Left / Right: Adjust Camera Yaw (+/- 0.2 deg)
            if (gamepad1.dpad_left && !prevLeft) {
                Vision.CAMERA_YAW_DEGREES += 0.2;
                needsRebuild = true;
            }
            if (gamepad1.dpad_right && !prevRight) {
                Vision.CAMERA_YAW_DEGREES -= 0.2;
                needsRebuild = true;
            }

            // Bumpers: Adjust Focal Length (+/- 2.0 px)
            if (gamepad1.right_bumper && !prevRb) {
                Vision.LENS_FX += 2.0;
                Vision.LENS_FY += 2.0;
                needsRebuild = true;
            }
            if (gamepad1.left_bumper && !prevLb) {
                Vision.LENS_FX -= 2.0;
                Vision.LENS_FY -= 2.0;
                needsRebuild = true;
            }

            // Button A: Reset Yaw to 0.0 (straight forward) & rebuild
            if (gamepad1.a && !prevA) {
                Vision.CAMERA_YAW_DEGREES = 0.0;
                needsRebuild = true;
            }

            Vision.INSTANCE.periodic();
            AprilTagClusterDetection primaryCluster = getFirstCluster(Vision.INSTANCE.getDetections());

            // Button Y: Auto-solve CAMERA_YAW_DEGREES assuming target cluster is dead-center ahead (rx = 0)
            if (gamepad1.y && !prevY && primaryCluster != null && primaryCluster.ftcPose != null) {
                AprilTagPoseFtc pose = primaryCluster.ftcPose;
                double pitch = Math.toRadians(Vision.CAMERA_UPWARD_TILT_DEGREES);
                double levelX = pose.x;
                double levelY = pose.y * Math.cos(pitch) - pose.z * Math.sin(pitch);

                // Solve: levelX * cos(yaw) - levelY * sin(yaw) + CAMERA_TO_CENTER_X_INCHES = 0
                double targetCamX = -Vision.CAMERA_TO_CENTER_X_INCHES;
                double angleToTarget = Math.atan2(levelX, levelY);
                double angleForOffset = Math.asin(
                        Math.max(-1.0, Math.min(1.0, targetCamX / Math.hypot(levelX, levelY)))
                );
                Vision.CAMERA_YAW_DEGREES = Math.toDegrees(angleToTarget - angleForOffset);
                needsRebuild = true;
            }

            // Button X: Record current calibration snapshot to Telemetry & RobotLog
            if (gamepad1.x && !prevX) {
                recordCount++;
                String measuredInfo = "No cluster in view";
                if (primaryCluster != null && primaryCluster.ftcPose != null) {
                    double[] rel = Vision.cameraToRobotCenter(primaryCluster.ftcPose);
                    measuredInfo = String.format(Locale.US, "RobotXYZ=(%.2f, %.2f, %.2f)", rel[0], rel[1], rel[2]);
                }

                lastRecordedSummary = String.format(Locale.US,
                        "#%d: Tilt=%.2f deg, Yaw=%.2f deg, fx=%.1f | %s",
                        recordCount,
                        Vision.CAMERA_UPWARD_TILT_DEGREES,
                        Vision.CAMERA_YAW_DEGREES,
                        Vision.LENS_FX,
                        measuredInfo);

                RobotLog.ii(LOG_TAG, "=== RECORDED CAMERA CALIBRATION #%d ===", recordCount);
                RobotLog.ii(LOG_TAG, "public static double CAMERA_TO_CENTER_X_INCHES = %.3f;", Vision.CAMERA_TO_CENTER_X_INCHES);
                RobotLog.ii(LOG_TAG, "public static double CAMERA_TO_CENTER_Y_INCHES = %.3f;", Vision.CAMERA_TO_CENTER_Y_INCHES);
                RobotLog.ii(LOG_TAG, "public static double CAMERA_TO_CENTER_Z_INCHES = %.3f;", Vision.CAMERA_TO_CENTER_Z_INCHES);
                RobotLog.ii(LOG_TAG, "public static double CAMERA_UPWARD_TILT_DEGREES = %.2f;", Vision.CAMERA_UPWARD_TILT_DEGREES);
                RobotLog.ii(LOG_TAG, "public static double CAMERA_YAW_DEGREES = %.2f;", Vision.CAMERA_YAW_DEGREES);
                RobotLog.ii(LOG_TAG, "public static double LENS_FX = %.1f; public static double LENS_FY = %.1f;", Vision.LENS_FX, Vision.LENS_FY);
                RobotLog.ii(LOG_TAG, "Verification Reading: %s", measuredInfo);
            }

            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevRb = gamepad1.right_bumper;
            prevLb = gamepad1.left_bumper;
            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevY = gamepad1.y;

            if (needsRebuild) {
                Vision.INSTANCE.rebuildPortal(hardwareMap);
            }

            telemetry.addLine("=== RECORDED RESULT [Press X to Save Snapshot] ===");
            telemetry.addData("Last Saved", lastRecordedSummary);
            telemetry.addLine("------------------------------------------------------");
            telemetry.addData("Mount XYZ (in)", "X=%.2f, Y=%.2f, Z=%.2f",
                    Vision.CAMERA_TO_CENTER_X_INCHES,
                    Vision.CAMERA_TO_CENTER_Y_INCHES,
                    Vision.CAMERA_TO_CENTER_Z_INCHES);
            telemetry.addData("Tilt / Yaw (deg)", "Tilt=%.2f [D-Pad U/D] | Yaw=%.2f [Y=Auto, A=0]",
                    Vision.CAMERA_UPWARD_TILT_DEGREES,
                    Vision.CAMERA_YAW_DEGREES);
            telemetry.addData("Resolution / Focal", "%dx%d  fx=%.1f, fy=%.1f  [LB/RB]",
                    Vision.CAMERA_WIDTH, Vision.CAMERA_HEIGHT,
                    Vision.LENS_FX, Vision.LENS_FY);
            telemetry.addLine("------------------------------------------------------");

            int clusterCount = 0;
            for (AprilTagDetection det : Vision.INSTANCE.getDetections()) {
                if (!(det instanceof AprilTagClusterDetection)) {
                    continue;
                }
                clusterCount++;
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) det;

                String label = (cluster.metadata != null && cluster.metadata.name != null)
                        ? cluster.metadata.name
                        : "Unknown Cluster";
                boolean scorable = Vision.INSTANCE.isCellScorable(cluster);

                telemetry.addLine(String.format(Locale.US, "Cluster: %s (Found: %d%%, Scorable: %b)",
                        label, cluster.percentClusterFound, scorable));

                if (cluster.ftcPose != null) {
                    telemetry.addData("  Raw Camera RBE", "Range=%.2f in | Bearing=%.2f deg | Elev=%.2f deg",
                            cluster.ftcPose.range, cluster.ftcPose.bearing, cluster.ftcPose.elevation);
                    telemetry.addData("  Raw Camera XYZ", "X=%.2f, Y=%.2f, Z=%.2f (Roll=%.1f deg)",
                            cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z, cluster.ftcPose.roll);

                    double[] targetRelRobot = Vision.cameraToRobotCenter(cluster.ftcPose);
                    double rx = targetRelRobot[0];
                    double ry = targetRelRobot[1];
                    double rz = targetRelRobot[2];
                    double turretAimDeg = Math.toDegrees(Math.atan2(rx, ry));

                    telemetry.addData("  Target Rel-to-Robot", "X(Right)=%.2f in | Y(Fwd)=%.2f in | Z(Up)=%.2f in",
                            rx, ry, rz);
                    telemetry.addData("  Turret Aim Angle", "%.2f deg (+ = Right, 0 = Straight)", turretAimDeg);
                }

                if (cluster.robotPose != null) {
                    telemetry.addData("  Field Robot Pose", "FieldX=%.2f in | FieldY=%.2f in | Heading=%.1f deg",
                            cluster.robotPose.getPosition().x,
                            cluster.robotPose.getPosition().y,
                            cluster.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                }
            }

            telemetry.addData("Clusters Detected", clusterCount);
            telemetry.update();
            sleep(20);
        }

        Vision.INSTANCE.close();
    }

    /**
     * Returns the first detected AprilTagClusterDetection, or null if none are visible.
     */
    private AprilTagClusterDetection getFirstCluster(List<AprilTagDetection> detections) {
        for (AprilTagDetection det : detections) {
            if (det instanceof AprilTagClusterDetection) {
                return (AprilTagClusterDetection) det;
            }
        }
        return null;
    }
}