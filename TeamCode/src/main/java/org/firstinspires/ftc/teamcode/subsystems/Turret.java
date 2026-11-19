package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Encoder-relative turret positioning supporting either a 1150-RPM or 435-RPM goBILDA motor.
 */
public final class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    public enum MotorOption {
        RPM_1150(1150.0, 145.1),
        RPM_435(435.0, 384.5);

        public final double ratedRpm;
        public final double defaultTicksPerRev;

        MotorOption(double ratedRpm, double defaultTicksPerRev) {
            this.ratedRpm = ratedRpm;
            this.defaultTicksPerRev = defaultTicksPerRev;
        }
    }

    public static MotorOption MOTOR_OPTION = MotorOption.RPM_435;
    public static double MOTOR_REVS_PER_TURRET_REV = 1.0; // External gear/belt ratio
    public static double MIN_ANGLE_DEG = -30.0;
    public static double MAX_ANGLE_DEG = 30.0;
    public static double MAX_POWER = 0.25;
    public static double KP = 0.012;  // Proportional gain (power per degree)
    public static double KD = 0.0008; // Derivative damping (power per deg/sec)
    public static double ANGLE_TOLERANCE_DEG = 1.5;

    private DcMotorEx motor;
    private int zeroTicks;
    private double targetAngleDeg;
    private boolean positionMode;
    private double manualPower;

    private Turret() {}

    public void init(DcMotorEx turretMotor) {
        if (turretMotor == null) throw new IllegalArgumentException("Turret motor cannot be null");
        motor = turretMotor;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        resetZero();
    }

    /** Sets the current physical turret angle as 0.0 degrees. */
    public void resetZero() {
        requireInitialized();
        zeroTicks = motor.getCurrentPosition();
        targetAngleDeg = 0.0;
        manualPower = 0.0;
        positionMode = false;
    }

    private void requireInitialized() {
        if (motor == null) throw new IllegalStateException("Turret.init() not called");
    }

    public double getTicksPerMotorRevolution() {
        requireInitialized();
        double ticks = motor.getMotorType().getTicksPerRev();
        return (Double.isFinite(ticks) && ticks > 0.0) ? ticks : MOTOR_OPTION.defaultTicksPerRev;
    }

    public double getTicksPerTurretDegree() {
        return (getTicksPerMotorRevolution() * MOTOR_REVS_PER_TURRET_REV) / 360.0;
    }

    public double getAngleDegrees() {
        requireInitialized();
        return (motor.getCurrentPosition() - zeroTicks) / getTicksPerTurretDegree();
    }

    public double getAngularVelocityDegPerSec() {
        requireInitialized();
        return motor.getVelocity() / getTicksPerTurretDegree();
    }

    public int getEncoderTicks() {
        requireInitialized();
        return motor.getCurrentPosition();
    }

    public void setTargetAngle(double degrees) {
        requireInitialized();
        targetAngleDeg = Range.clip(degrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        positionMode = true;
        manualPower = 0;
    }

    public double getTargetAngle() { return targetAngleDeg; }

    public boolean isAtTarget() {
        return positionMode && Math.abs(targetAngleDeg - getAngleDegrees()) <= ANGLE_TOLERANCE_DEG;
    }

    public void setManualPower(double power) {
        requireInitialized();
        positionMode = false;
        manualPower = Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    public void stop() {
        if (motor != null) motor.setPower(0);
        manualPower = 0;
        positionMode = false;
    }

    public boolean isPositionMode() { return positionMode; }

    public Command setTargetAngleCommand(double degrees) {
        return new InstantCommand(() -> setTargetAngle(degrees));
    }

    @Override
    public void periodic() {
        if (motor == null) return;
        double angle = getAngleDegrees();
        double power;
        if (positionMode) {
            double error = targetAngleDeg - angle;
            if (Math.abs(error) <= ANGLE_TOLERANCE_DEG) {
                power = 0.0;
            } else {
                double velDegPerSec = getAngularVelocityDegPerSec();
                power = Range.clip((KP * error) - (KD * velDegPerSec), -MAX_POWER, MAX_POWER);
            }
        } else {
            power = manualPower;
        }
        if (angle <= MIN_ANGLE_DEG && power < 0) power = 0;
        if (angle >= MAX_ANGLE_DEG && power > 0) power = 0;
        motor.setPower(power);
    }
}