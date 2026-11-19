package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/** Torque/Speed positional servo gate preventing balls from entering the flywheel too soon. */
public final class Blocker implements Subsystem {
    public static final Blocker INSTANCE = new Blocker();

    // PLACEHOLDERS: calibrate safe positions using ShooterCalibrationTest.
    public static double CLOSED_POSITION = 0.50;
    public static double OPEN_POSITION = 0.60;
    public static double SETTLE_SECONDS = 0.15;

    private Servo servo;
    private boolean open;
    private double commandedPosition = CLOSED_POSITION;
    private final ElapsedTime moveTimer = new ElapsedTime();

    private Blocker() {}

    public void init(Servo servo) {
        if (servo == null) throw new IllegalArgumentException("Blocker servo is null");
        this.servo = servo;
        close();
    }

    public void setPosition(double position) {
        if (servo == null) throw new IllegalStateException("Blocker.init() required");
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Invalid blocker servo position");
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

    public void open() {
        setPosition(OPEN_POSITION);
        open = true;
    }

    public void close() {
        setPosition(CLOSED_POSITION);
        open = false;
    }

    public boolean isOpen() { return open; }
    public boolean isReady() { return servo != null && moveTimer.seconds() >= SETTLE_SECONDS; }
    public double getPosition() { return commandedPosition; }

    public Command openCommand() { return new InstantCommand(this::open); }
    public Command closeCommand() { return new InstantCommand(this::close); }

    @Override public void periodic() {}
}