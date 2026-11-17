package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Odom;

/**
 * Calibration OpMode for the goBILDA Pinpoint Odometry Computer ("odom")
 * after relocating the odometry pods on the chassis.
 *
 * Controls (Gamepad 1):
 * - Button A: Reset pose and IMU to (0, 0, 0)
 * - Button B: Start / Snapshot Pod Offset Spin Calibration (spin robot 3-5 full turns in place)
 * - Button X: Flip X Pod Encoder Direction
 * - Button Y: Flip Y Pod Encoder Direction
 * - D-Pad Up / Down: Adjust Yaw Scalar (+/- 0.002) if 360-deg spin heading drifts
 */
@TeleOp(name = "Odometry Calibration Test", group = "Calibration")
public class TestOdom extends LinearOpMode {

    // goBILDA 4-Bar Odometry Pod ticks per inch (~19.89436789 ticks/mm * 25.4 mm/in)
    private static final double TICKS_PER_INCH = 505.31694;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    private boolean spinCalibrating = false;
    private int startEncoderX = 0;
    private int startEncoderY = 0;
    private double startHeadingRad = 0.0;
    private double accumulatedHeadingRad = 0.0;
    private double lastHeadingRad = 0.0;

    private double calculatedXOffsetIn = Odom.X_POD_OFFSET_INCHES;
    private double calculatedYOffsetIn = Odom.Y_POD_OFFSET_INCHES;
    private double yawScalar = 1.0;

    @Override
    public void runOpMode() {
        Odom.INSTANCE.init(hardwareMap, telemetry);
        Odom.INSTANCE.setEnableTelemetry(false);
        GoBildaPinpointDriver pinpoint = Odom.INSTANCE.getPinpoint();

        telemetry.addLine("Pinpoint Odometry Calibration Ready.");
        telemetry.addLine("Keep robot completely still during init for IMU calibration.");
        telemetry.update();

        waitForStart();
        Odom.INSTANCE.resetPosition();

        while (opModeIsActive()) {
            pinpoint.update();

            // Track unwrapped total heading for multi-turn spin calibration
            double currentHeadingRad = pinpoint.getHeading(AngleUnit.RADIANS);
            double deltaHeading = Math.atan2(
                    Math.sin(currentHeadingRad - lastHeadingRad),
                    Math.cos(currentHeadingRad - lastHeadingRad)
            );
            accumulatedHeadingRad += deltaHeading;
            lastHeadingRad = currentHeadingRad;

            // Button A: Reset Pose & Recalibrate IMU
            if (gamepad1.a && !prevA) {
                pinpoint.recalibrateIMU();
                Odom.INSTANCE.resetPosition();
                accumulatedHeadingRad = 0.0;
                spinCalibrating = false;
            }

            // Button B: Toggle Spin Offset Calibration
            if (gamepad1.b && !prevB) {
                if (!spinCalibrating) {
                    spinCalibrating = true;
                    startEncoderX = pinpoint.getEncoderX();
                    startEncoderY = pinpoint.getEncoderY();
                    startHeadingRad = accumulatedHeadingRad;
                } else {
                    spinCalibrating = false;
                    double totalTurnRad = accumulatedHeadingRad - startHeadingRad;
                    if (Math.abs(totalTurnRad) > Math.PI) {
                        double deltaXInches = (pinpoint.getEncoderX() - startEncoderX) / TICKS_PER_INCH;
                        double deltaYInches = (pinpoint.getEncoderY() - startEncoderY) / TICKS_PER_INCH;

                        // Arc length s = r * theta  =>  offset = travel / totalAngleRadians
                        calculatedXOffsetIn = deltaXInches / totalTurnRad;
                        calculatedYOffsetIn = deltaYInches / totalTurnRad;

                        Odom.X_POD_OFFSET_INCHES = calculatedXOffsetIn;
                        Odom.Y_POD_OFFSET_INCHES = calculatedYOffsetIn;
                        Odom.INSTANCE.applyConfiguration();
                    }
                }
            }

            // Button X: Toggle X Pod Direction
            if (gamepad1.x && !prevX) {
                Odom.X_POD_DIRECTION = (Odom.X_POD_DIRECTION == GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD;
                Odom.INSTANCE.applyConfiguration();
            }

            // Button Y: Toggle Y Pod Direction
            if (gamepad1.y && !prevY) {
                Odom.Y_POD_DIRECTION = (Odom.Y_POD_DIRECTION == GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD;
                Odom.INSTANCE.applyConfiguration();
            }

            // D-Pad Up / Down: Tune Yaw Scalar
            if (gamepad1.dpad_up && !prevUp) {
                yawScalar += 0.002;
                pinpoint.setYawScalar(yawScalar);
            }
            if (gamepad1.dpad_down && !prevDown) {
                yawScalar -= 0.002;
                pinpoint.setYawScalar(yawScalar);
            }

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            telemetry.addLine("=== PINPOINT LIVE POSE ===");
            telemetry.addData("Pose X (in) [Strafe/Right]", "%.3f", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Pose Y (in) [Forward]", "%.3f", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.2f (Total Turns: %.2f)",
                    pinpoint.getHeading(AngleUnit.DEGREES),
                    accumulatedHeadingRad / (2.0 * Math.PI));
            telemetry.addData("Device Status", pinpoint.getDeviceStatus());

            telemetry.addLine("\n=== ENCODER DIRECTIONS [X / Y Buttons] ===");
            telemetry.addData("X Pod Direction", "%s (Raw Ticks: %d)", Odom.X_POD_DIRECTION, pinpoint.getEncoderX());
            telemetry.addData("Y Pod Direction", "%s (Raw Ticks: %d)", Odom.Y_POD_DIRECTION, pinpoint.getEncoderY());
            telemetry.addData("Yaw Scalar [D-Pad U/D]", "%.4f", yawScalar);

            telemetry.addLine("\n=== POD OFFSETS [B = Start/Stop Spin Cal] ===");
            telemetry.addData("Spin Calibration Active", spinCalibrating ? "YES - Spin robot 4 turns!" : "No");
            telemetry.addData("X_POD_OFFSET_INCHES", "%.4f in", calculatedXOffsetIn);
            telemetry.addData("Y_POD_OFFSET_INCHES", "%.4f in", calculatedYOffsetIn);

            telemetry.update();
            sleep(20);
        }
    }
}