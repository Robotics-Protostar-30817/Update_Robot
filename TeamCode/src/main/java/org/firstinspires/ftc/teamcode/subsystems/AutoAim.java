package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Stationary, vision-guided aiming.
 * Initialize Vision, Turret, Hood, Flywheel, and Blocker first. Never actuates Transfer.
 * CELL coordinates must be validated as robot-centered inches; turret zero is forward.
 */
public final class AutoAim implements Subsystem {
    public static final AutoAim INSTANCE = new AutoAim();

    // COPY VERIFIED SUCCESSFUL ROWS from ShotCalibration here, sorted by distance.
    // Each row: {horizontal distance in inches, physical hood launch angle (degrees), flywheel RPM}.
    // At least two distinct distance rows required. No extrapolation.
    public static final double[][] SHOT_TABLE = {
            {36.0, Double.NaN, 2500.0}, // Replace angle with measured successful shot
            {60.0, Double.NaN, 3000.0}, // Replace angle with measured successful shot
    };
    public static boolean CALIBRATION_VERIFIED = false; // Set true after filling verified values
    public static double TURRET_DIRECTION = 1.0;        // Set -1.0 if positive angle turns left
    public static double TURRET_FORWARD_OFFSET_DEG = 0.0;
    public static double MAX_TARGET_CHANGE_DEG = 5.0;   // Optional telemetry reference only

    private final Vision vision = Vision.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Blocker blocker = Blocker.INSTANCE;

    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private Vision.Cell.Location targetLocation = null; // null = any scorable cell of alliance color
    private boolean autoCloseBlockerOnStop = true;
    private boolean callSubPeriodic = false; // Keep false when using NextFTC OpMode scheduler

    private boolean enabled;
    private boolean ready;
    private String status = "Disabled";
    private String trackedCellName = "None";
    private double distanceInches = Double.NaN;
    private double targetTurretDeg = Double.NaN;
    private double targetHoodPosition = Double.NaN;
    private double targetHoodAngleDegrees = Double.NaN;
    private double targetRpm = Double.NaN;

    private AutoAim() { }

    /** Initialize aiming for any scorable cell matching the given alliance color. */
    public void init(Vision.Cell.Color color) {
        init(color, null);
    }

    /** Initialize aiming for a specific alliance color and field location (SCORING, AUDIENCE, or null for any). */
    public void init(Vision.Cell.Color color, Vision.Cell.Location location) {
        if (color != Vision.Cell.Color.RED && color != Vision.Cell.Color.BLUE) {
            throw new IllegalArgumentException("Alliance must be RED or BLUE");
        }
        alliance = color;
        targetLocation = location;
        enabled = false;
        ready = false;
        status = "Disabled";
        trackedCellName = "None";
        hood.invalidateReadiness();
        distanceInches = targetTurretDeg = targetHoodPosition = targetHoodAngleDegrees = targetRpm = Double.NaN;
    }

    public void setTargetLocation(Vision.Cell.Location location) {
        this.targetLocation = location;
    }

    public Vision.Cell.Location getTargetLocation() {
        return targetLocation;
    }

    public void setAutoCloseBlockerOnStop(boolean autoClose) {
        this.autoCloseBlockerOnStop = autoClose;
    }

    /**
     * Set true ONLY if calling AutoAim.periodic() manually in a standard LinearOpMode/OpMode
     * without NextFTC's SubsystemManager also calling periodic() on Turret, Hood, and Flywheel.
     */
    public void setCallSubPeriodic(boolean callSubPeriodic) {
        this.callSubPeriodic = callSubPeriodic;
    }

    public void enable() { enabled = true; }

    public void disable() {
        enabled = false;
        safeStop("Disabled");
    }

    public Command enableCommand() { return new InstantCommand(this::enable); }
    public Command disableCommand() { return new InstantCommand(this::disable); }

    public boolean isEnabled() { return enabled; }
    /** Readiness is approximate: positional servos have no hood position feedback. */
    public boolean isReadyToShoot() { return ready; }
    public String getStatus() { return status; }
    public String getTrackedCellName() { return trackedCellName; }
    public double getDistanceInches() { return distanceInches; }
    public double getTargetTurretDegrees() { return targetTurretDeg; }
    public double getTargetHoodPosition() { return targetHoodPosition; }
    public double getTargetHoodAngleDegrees() { return targetHoodAngleDegrees; }
    public double getTargetFlywheelRPM() { return targetRpm; }

    private void safeStop(String reason) {
        ready = false;
        hood.invalidateReadiness();
        status = reason;
        trackedCellName = "None";
        distanceInches = targetTurretDeg = targetHoodPosition = targetHoodAngleDegrees = targetRpm = Double.NaN;
        turret.stop();
        flywheel.stop();
        if (autoCloseBlockerOnStop) {
            try {
                blocker.close();
            } catch (IllegalStateException ignored) {
                // Blocker not initialized in standalone bench tests
            }
        }
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        Vision.Cell cell = (targetLocation == null)
                ? vision.getBestCell(alliance)
                : vision.getBestCell(alliance, targetLocation);

        if (cell == null || !cell.isScorable()) {
            safeStop("No scorable CELL");
            return;
        }
        trackedCellName = cell.getNameAndColor();

        double x = cell.getX(), y = cell.getY(), z = cell.getZ();
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
            safeStop("Invalid CELL XYZ");
            return;
        }

        double distance = Math.hypot(x, y);
        if (distance <= 1.0) {
            safeStop("Invalid distance");
            return;
        }

        double direction = TURRET_DIRECTION * Math.toDegrees(Math.atan2(x, y))
                + TURRET_FORWARD_OFFSET_DEG;
        if (!Double.isFinite(direction) || direction < Turret.MIN_ANGLE_DEG
                || direction > Turret.MAX_ANGLE_DEG) {
            safeStop("CELL outside turret limits");
            return;
        }

        if (!CALIBRATION_VERIFIED || !validTable()) {
            safeStop("Shot table missing/unverified");
            return;
        }
        if (!hood.hasValidAngleCalibration()) {
            safeStop("Hood calibration missing/unverified");
            return;
        }
        if (distance < SHOT_TABLE[0][0] || distance > SHOT_TABLE[SHOT_TABLE.length - 1][0]) {
            safeStop("Distance outside calibrated range");
            return;
        }

        double hoodAngle = Double.NaN, rpm = Double.NaN;
        for (int i = 0; i < SHOT_TABLE.length - 1; i++) {
            double[] a = SHOT_TABLE[i], b = SHOT_TABLE[i + 1];
            if (distance >= a[0] && distance <= b[0]) {
                double fraction = (distance - a[0]) / (b[0] - a[0]);
                hoodAngle = a[1] + fraction * (b[1] - a[1]);
                rpm = a[2] + fraction * (b[2] - a[2]);
                break;
            }
        }

        double hoodPos;
        try {
            hoodPos = hood.servoPositionForAngle(hoodAngle);
        } catch (IllegalArgumentException | IllegalStateException ex) {
            safeStop("Hood angle outside calibration");
            return;
        }

        if (!Double.isFinite(hoodPos) || !Double.isFinite(rpm)
                || rpm <= 0 || rpm > Flywheel.RATED_MOTOR_RPM) {
            safeStop("Shot command outside limits");
            return;
        }

        distanceInches = distance;
        targetTurretDeg = direction;
        targetHoodPosition = hoodPos;
        targetHoodAngleDegrees = hoodAngle;
        targetRpm = rpm;

        turret.setTargetAngle(direction);
        hood.setLaunchAngle(hoodAngle);
        flywheel.setFlywheelRPM(rpm);

        if (callSubPeriodic) {
            turret.periodic();
            hood.periodic();
            flywheel.periodic();
        }

        boolean turretReady = turret.isAtTarget();
        boolean flywheelReady = flywheel.isReadyToShoot();
        boolean hoodReady = hood.isReady(); // Time-estimated, not position-measured.
        ready = turretReady && flywheelReady && hoodReady;

        if (ready) status = "Ready to shoot (" + trackedCellName + ")";
        else if (!turretReady) status = "Waiting for turret";
        else if (!flywheelReady) status = "Waiting for flywheel";
        else status = "Waiting for hood to settle";
    }

    private boolean validTable() {
        if (SHOT_TABLE.length < 2) return false;
        double previousDistance = Double.NEGATIVE_INFINITY;
        for (double[] row : SHOT_TABLE) {
            if (row == null || row.length != 3 || !Double.isFinite(row[0])
                    || !Double.isFinite(row[1]) || !Double.isFinite(row[2])
                    || row[0] <= previousDistance || row[0] <= 0
                    || row[2] <= 0 || row[2] > Flywheel.RATED_MOTOR_RPM) {
                return false;
            }
            previousDistance = row[0];
        }
        return true;
    }
}