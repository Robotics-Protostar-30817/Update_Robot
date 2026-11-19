package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/** Torque/Speed positional servo that raises/lowers the floor intake ramp. */
public final class IntakeRamp implements Subsystem {
    public static final IntakeRamp INSTANCE = new IntakeRamp();

    // PLACEHOLDERS: calibrate with linkage disconnected or via ShooterCalibrationTest.
    public static double RAISED_POSITION = 0.50;
    public static double LOWERED_POSITION = 0.60;
    public static double SETTLE_SECONDS = 0.25;

    private Servo servo;
    private boolean lowered;
    private double commandedPosition = RAISED_POSITION;
    private final ElapsedTime moveTimer = new ElapsedTime();

    private IntakeRamp() {}

    public void init(Servo rampServo) {
        if (rampServo == null) {
            throw new IllegalArgumentException("Intake ramp servo is null");
        }
        servo = rampServo;
        lowered = false;
        raise();
    }

    public void setPosition(double position) {
        if (servo == null) {
            throw new IllegalStateException("IntakeRamp.init() required");
        }
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Invalid intake ramp servo position");
        }
        double clipped = Range.clip(position, 0.0, 1.0);
        if (Math.abs(clipped - commandedPosition) > 0.002) {
            moveTimer.reset();
        }
        commandedPosition = clipped;
        servo.setPosition(commandedPosition);
    }

    public void adjustPosition(double delta) {
        setPosition(commandedPosition + delta);
    }

    public void lower() {
        setPosition(LOWERED_POSITION);
        lowered = true;
    }

    public void raise() {
        setPosition(RAISED_POSITION);
        lowered = false;
    }

    public void intake() { lower(); }
    public void stop() { if (servo != null) raise(); }

    public boolean isLowered() { return lowered; }
    public boolean isReady() { return servo != null && moveTimer.seconds() >= SETTLE_SECONDS; }
    public double getPosition() { return commandedPosition; }

    public Command lowerCommand() { return new InstantCommand(this::lower); }
    public Command raiseCommand() { return new InstantCommand(this::raise); }

    @Override public void periodic() {}
}