package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Odometry subsystem wrapping the goBILDA Pinpoint Odometry Computer ("odom").
 * Stores single-source-of-truth calibration constants shared with Pedro Pathing's Constants.java.
 */
public class Odom implements Subsystem {

    public static final Odom INSTANCE = new Odom();

    // -------------------------------------------------------------------------
    // SINGLE-SOURCE-OF-TRUTH PINPOINT CALIBRATION CONSTANTS (INCHES)
    // Update these after running OdomCalibrationOpMode and reference in Constants.java
    // -------------------------------------------------------------------------
    public static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // X Offset: Forward/backward distance from robot center to the lateral (strafe) pod wheel (inches)
    public static double X_POD_OFFSET_INCHES = -4.5;

    // Y Offset: Left/right distance from robot center to the forward (axial) pod wheel (inches)
    public static double Y_POD_OFFSET_INCHES = -0.5;

    public static GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;
    private boolean enableTelemetry = false;

    private Odom() {}

    /**
     * Initializes the Pinpoint driver for TeleOp / standalone odometry tracking.
     * Note: Do NOT call Odom.periodic() in Autonomous if Pedro Pathing's PinpointLocalizer is active.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odom");

        applyConfiguration();
        pinpoint.recalibrateIMU();
    }

    /**
     * Applies pod resolution, physical offsets, and encoder directions to the Pinpoint.
     */
    public void applyConfiguration() {
        if (pinpoint == null) return;
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setOffsets(X_POD_OFFSET_INCHES, Y_POD_OFFSET_INCHES, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
    }

    public void setEnableTelemetry(boolean enable) {
        this.enableTelemetry = enable;
    }

    @Override
    public void periodic() {
        if (pinpoint == null) return;

        pinpoint.update();

        if (enableTelemetry && telemetry != null) {
            telemetry.addData("Odom X (in)", "%.2f", getXInches());
            telemetry.addData("Odom Y (in)", "%.2f", getYInches());
            telemetry.addData("Odom Heading (deg)", "%.2f", getHeadingDegrees());
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            // Do NOT call telemetry.update() here
        }
    }

    public double getXInches() {
        return pinpoint != null ? pinpoint.getPosX(DistanceUnit.INCH) : 0.0;
    }

    public double getYInches() {
        return pinpoint != null ? pinpoint.getPosY(DistanceUnit.INCH) : 0.0;
    }

    public double getHeadingDegrees() {
        return pinpoint != null ? pinpoint.getHeading(AngleUnit.DEGREES) : 0.0;
    }

    public double getHeadingRadians() {
        return pinpoint != null ? pinpoint.getHeading(AngleUnit.RADIANS) : 0.0;
    }

    public Pose2D getPosition() {
        return pinpoint != null ? pinpoint.getPosition() : null;
    }

    public void setPosition(double xInches, double yInches, double headingDegrees) {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees
            ));
        }
    }

    public void resetPosition() {
        setPosition(0.0, 0.0, 0.0);
    }

    public Command resetPositionCommand() {
        return new InstantCommand(this::resetPosition);
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }
}