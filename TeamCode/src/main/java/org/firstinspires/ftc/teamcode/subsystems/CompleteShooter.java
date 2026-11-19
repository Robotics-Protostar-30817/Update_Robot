package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Coordinates floor intake, intake ramp, vision-guided aiming, blocker gate, and upper feed motor.
 * Continuous feed does NOT detect when storage is empty.
 */
public final class CompleteShooter implements Subsystem {
    public static final CompleteShooter INSTANCE = new CompleteShooter();

    // All values are starting points; update after running ShooterCalibrationTest.
    public static double INTAKE_POWER = 0.25;
    public static double FEED_POWER = 0.25;
    public static double IDLE_FLYWHEEL_RPM = 800.0;
    public static double BLOCKER_OPEN_SECONDS = 0.20;

    // Optional watchdogs: 0 disables the timeout (default).
    public static double MAX_CONTINUOUS_FEED_SECONDS = 0.0;
    public static double REQUEST_TIMEOUT_SECONDS = 0.0;

    private enum Stage { IDLE, WAITING, OPENING, FEEDING }
    private Stage stage = Stage.IDLE;

    private final AutoAim aim = AutoAim.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Transfer transfer = Transfer.INSTANCE;
    private final Blocker blocker = Blocker.INSTANCE;
    private final IntakeRamp ramp = IntakeRamp.INSTANCE;

    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime requestTimer = new ElapsedTime();

    private boolean initialized = false;
    private boolean collecting = false;
    private boolean callSubPeriodic = false; // Keep false when NextFTC scheduler updates subsystems
    private String status = "Uninitialized";

    private CompleteShooter() {}

    /** Call after Vision, Turret, Hood, Flywheel, Transfer, Blocker, and IntakeRamp init(). */
    public void init(Vision.Cell.Color alliance) {
        init(alliance, null);
    }

    /** Initialize with alliance color and optional target location (SCORING, AUDIENCE, or null for any). */
    public void init(Vision.Cell.Color alliance, Vision.Cell.Location location) {
        transfer.stop();
        ramp.raise();
        blocker.close();
        flywheel.stop();
        aim.init(alliance, location);
        aim.setCallSubPeriodic(callSubPeriodic);
        collecting = false;
        stage = Stage.IDLE;
        initialized = true;
        status = "Idle";
    }

    /**
     * Set true ONLY if calling CompleteShooter.periodic() manually in a standard OpMode/LinearOpMode
     * without NextFTC's SubsystemManager also calling periodic() on AutoAim and Flywheel.
     */
    public void setCallSubPeriodic(boolean callSubPeriodic) {
        this.callSubPeriodic = callSubPeriodic;
        aim.setCallSubPeriodic(callSubPeriodic);
    }

    public void setTargetLocation(Vision.Cell.Location location) {
        requireInit();
        aim.setTargetLocation(location);
    }

    private void requireInit() {
        if (!initialized) throw new IllegalStateException("CompleteShooter.init() required");
    }

    /** Lower transfer motor + ramp only. Upper feed motor and blocker remain independent. */
    public void startCollecting() {
        requireInit();
        if (!Double.isFinite(INTAKE_POWER) || INTAKE_POWER <= 0 || INTAKE_POWER > 1) {
            throw new IllegalStateException("Invalid intake power");
        }
        transfer.setIntakePower(INTAKE_POWER);
        ramp.lower();
        collecting = true;
    }

    public void stopCollecting() {
        if (!initialized) return;
        transfer.stopIntake();
        ramp.raise();
        collecting = false;
    }

    public boolean isCollecting() { return collecting; }

    public void enableAim() {
        requireInit();
        aim.enable();
    }

    public void disableAim() {
        requireInit();
        cancelShot();
        aim.disable();
    }

    /**
     * One-button command: enable AutoAim and start continuous shooting once ready.
     * Continues until driver calls cancelShot(), or a safety fault occurs.
     * Does not interrupt floor collection.
     */
    public void requestShot() {
        requireInit();
        if (stage != Stage.IDLE) return;
        aim.enable();
        stage = Stage.WAITING;
        requestTimer.reset();
        status = "Continuous shooting requested; waiting for aim";
    }

    public void cancelShot() {
        if (!initialized) return;
        transfer.stopFeed();
        blocker.close();
        stage = Stage.IDLE;
        status = "Shooting stopped by driver/cancel";
    }

    public boolean isFiring() { return stage == Stage.FEEDING; }
    public boolean isShootingActive() { return stage != Stage.IDLE; }
    public boolean isAimReady() { return initialized && aim.isReadyToShoot(); }
    public boolean canDetectEmptyStorage() { return false; }
    public boolean isShotPending() { return stage == Stage.WAITING || stage == Stage.OPENING; }
    public boolean isReadyToShoot() { return isAimReady(); }
    public String getStage() { return stage.name(); }
    public String getStatus() { return status; }
    public String getTrackedCellName() { return aim.getTrackedCellName(); }

    // -------------------------------------------------------------------------
    // NextFTC Command Factories
    // -------------------------------------------------------------------------
    public Command startCollectingCommand() { return new InstantCommand(this::startCollecting); }
    public Command stopCollectingCommand() { return new InstantCommand(this::stopCollecting); }
    public Command enableAimCommand() { return new InstantCommand(this::enableAim); }
    public Command disableAimCommand() { return new InstantCommand(this::disableAim); }
    public Command requestShotCommand() { return new InstantCommand(this::requestShot); }
    public Command cancelShotCommand() { return new InstantCommand(this::cancelShot); }
    public Command stopAllCommand() { return new InstantCommand(this::stop); }

    private boolean validFeedSettings() {
        return Double.isFinite(FEED_POWER) && FEED_POWER > 0 && FEED_POWER <= 1
                && Double.isFinite(BLOCKER_OPEN_SECONDS) && BLOCKER_OPEN_SECONDS >= 0
                && Double.isFinite(MAX_CONTINUOUS_FEED_SECONDS) && MAX_CONTINUOUS_FEED_SECONDS >= 0
                && Double.isFinite(REQUEST_TIMEOUT_SECONDS) && REQUEST_TIMEOUT_SECONDS >= 0;
    }

    @Override
    public void periodic() {
        if (!initialized) return;

        if (callSubPeriodic) {
            aim.periodic();
        }

        // Standby flywheel speed is commanded only when AutoAim is disabled, so it never
        // overrides an active aiming command or AutoAim safety stop.
        if (!aim.isEnabled()) {
            if (Double.isFinite(IDLE_FLYWHEEL_RPM) && IDLE_FLYWHEEL_RPM > 0
                    && IDLE_FLYWHEEL_RPM <= Flywheel.RATED_MOTOR_RPM) {
                flywheel.setFlywheelRPM(IDLE_FLYWHEEL_RPM);
                if (callSubPeriodic) {
                    flywheel.periodic();
                }
            } else {
                flywheel.stop();
            }
        }

        switch (stage) {
            case IDLE:
                transfer.stopFeed();
                blocker.close();
                status = aim.isEnabled() ? aim.getStatus() : "Idle (flywheel standby)";
                break;

            case WAITING:
                transfer.stopFeed();
                blocker.close();
                if (!aim.isEnabled()) {
                    cancelShot();
                    status = "Aim disabled";
                } else if (REQUEST_TIMEOUT_SECONDS > 0
                        && requestTimer.seconds() >= REQUEST_TIMEOUT_SECONDS) {
                    cancelShot();
                    status = "Shot request timed out";
                } else if (aim.isReadyToShoot()) {
                    if (!validFeedSettings()) {
                        cancelShot();
                        status = "Invalid feed calibration";
                    } else {
                        blocker.open();
                        stageTimer.reset();
                        stage = Stage.OPENING;
                        status = "Opening blocker";
                    }
                } else {
                    status = "Waiting: " + aim.getStatus();
                }
                break;

            case OPENING:
                if (!aim.isEnabled() || !aim.isReadyToShoot()) {
                    cancelShot();
                    status = "Aim not ready; blocker closed";
                } else if (blocker.isReady() && stageTimer.seconds() >= BLOCKER_OPEN_SECONDS) {
                    transfer.setFeedPower(FEED_POWER);
                    stageTimer.reset();
                    stage = Stage.FEEDING;
                    status = "Feeding toward flywheel (" + aim.getTrackedCellName() + ")";
                }
                break;

            case FEEDING:
                // Keep feeding multiple balls during one request; brief flywheel RPM dips
                // during launch do not terminate the cycle.
                if (!aim.isEnabled()) {
                    cancelShot();
                    status = "Target lost; continuous feed stopped";
                } else if (MAX_CONTINUOUS_FEED_SECONDS > 0
                        && stageTimer.seconds() >= MAX_CONTINUOUS_FEED_SECONDS) {
                    cancelShot();
                    status = "Feed safety timeout; empty storage NOT confirmed";
                } else {
                    status = "Continuous feeding (" + aim.getTrackedCellName() + ")";
                }
                break;
        }
    }

    /** Emergency stop: stops intake, feed, aim, and flywheel (no standby after stop()). */
    public void stop() {
        if (!initialized) return;
        cancelShot();
        stopCollecting();
        aim.disable();
        flywheel.stop();
        initialized = false;
        status = "Stopped";
    }
}