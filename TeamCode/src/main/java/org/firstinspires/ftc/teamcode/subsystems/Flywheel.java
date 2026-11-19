package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Single-motor flywheel subsystem for a 6000-RPM goBILDA Yellow Jacket Motor.
 */
public final class Flywheel implements Subsystem {

    public static final Flywheel INSTANCE = new Flywheel();

    public static final double RATED_MOTOR_RPM = 6000.0;
    public static final double TICKS_PER_REVOLUTION = 28.0; // Bare 6000-RPM motor encoder CPR
    public static final double BELT_RATIO = 1.0;

    public static double RPM_TOLERANCE = 150.0;
    public static double READY_STABLE_SECONDS = 0.2;

    private DcMotorEx motor;
    private double targetPower;
    private double targetVelocity;
    private double velocityTolerance = 100.0;
    private boolean velocityMode;

    private double targetFlywheelRPM = 0.0;
    private double readySinceSeconds = -1.0;
    private boolean readyToShoot = false;

    private final ElapsedTime timer = new ElapsedTime();

    private Flywheel() { }

    public void init(DcMotorEx flywheelMotor) {
        if (flywheelMotor == null) {
            throw new IllegalArgumentException("Flywheel motor cannot be null");
        }
        motor = flywheelMotor;
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPower = 0.0;
        targetVelocity = 0.0;
        targetFlywheelRPM = 0.0;
        velocityMode = false;
        readyToShoot = false;
        readySinceSeconds = -1.0;
    }

    private void requireInitialized() {
        if (motor == null) {
            throw new IllegalStateException("Call Flywheel.init() first");
        }
    }

    /** Direct motor-power control for bench testing (0.0 to 1.0). */
    public void setPower(double power) {
        requireInitialized();
        if (!Double.isFinite(power)) {
            throw new IllegalArgumentException("Power must be finite");
        }
        targetPower = Math.max(0.0, Math.min(1.0, power));
        targetVelocity = 0.0;
        targetFlywheelRPM = 0.0;
        velocityMode = false;
        readyToShoot = false;
        readySinceSeconds = -1.0;
        motor.setPower(targetPower);
    }

    public void setVelocity(double ticksPerSecond) {
        requireInitialized();
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond < 0.0) {
            throw new IllegalArgumentException("Velocity must be finite and nonnegative");
        }
        targetVelocity = ticksPerSecond;
        targetPower = 0.0;
        velocityMode = ticksPerSecond > 0.0;
        if (velocityMode) {
            motor.setVelocity(ticksPerSecond);
        } else {
            motor.setPower(0.0);
        }
    }

    public double getVelocity() {
        return motor == null ? 0.0 : motor.getVelocity();
    }

    public double getTicksPerRevolution() {
        requireInitialized();
        double ticks = motor.getMotorType().getTicksPerRev();
        return (Double.isFinite(ticks) && ticks > 0.0) ? ticks : TICKS_PER_REVOLUTION;
    }

    public double getMotorRPM() {
        return getVelocity() * 60.0 / getTicksPerRevolution();
    }

    public double getFlywheelRPM() {
        return getMotorRPM() * BELT_RATIO;
    }

    public void setMotorRPM(double rpm) {
        if (!Double.isFinite(rpm) || rpm < 0.0) {
            throw new IllegalArgumentException("RPM must be finite and nonnegative");
        }
        setVelocity(rpm * getTicksPerRevolution() / 60.0);
    }

    public void setFlywheelRPM(double rpm) {
        if (motor == null) return;
        double requestedRPM = Math.max(0.0, Math.min(RATED_MOTOR_RPM, rpm));

        if (Math.abs(requestedRPM - targetFlywheelRPM) > 1.0) {
            readyToShoot = false;
            readySinceSeconds = -1.0;
        }
        targetFlywheelRPM = requestedRPM;
        setMotorRPM(requestedRPM / BELT_RATIO);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getTargetPower() { return targetPower; }
    public boolean isVelocityMode() { return velocityMode; }

    public void setVelocityTolerance(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond <= 0.0) {
            throw new IllegalArgumentException("Tolerance must be positive and finite");
        }
        velocityTolerance = ticksPerSecond;
    }

    public boolean isAtSpeed() {
        return motor != null && velocityMode && targetVelocity > 0.0
                && Math.abs(getVelocity() - targetVelocity) <= velocityTolerance;
    }

    public void stop() {
        targetPower = 0.0;
        targetVelocity = 0.0;
        velocityMode = false;
        targetFlywheelRPM = 0.0;
        readyToShoot = false;
        readySinceSeconds = -1.0;
        if (motor != null) {
            motor.setPower(0.0);
        }
    }

    public boolean isReadyToShoot() { return readyToShoot; }
    public double getTargetFlywheelRPM() { return targetFlywheelRPM; }

    public Command spinUpCommand(double rpm) {
        return new InstantCommand(() -> setFlywheelRPM(rpm));
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop);
    }

    @Override
    public void periodic() {
        if (motor == null || targetFlywheelRPM <= 0.0) {
            readyToShoot = false;
            readySinceSeconds = -1.0;
            return;
        }

        double error = Math.abs(targetFlywheelRPM - getFlywheelRPM());
        if (error <= RPM_TOLERANCE) {
            if (readySinceSeconds < 0.0) {
                readySinceSeconds = timer.seconds();
            }
            readyToShoot = (timer.seconds() - readySinceSeconds) >= READY_STABLE_SECONDS;
        } else {
            readySinceSeconds = -1.0;
            readyToShoot = false;
        }
    }
}