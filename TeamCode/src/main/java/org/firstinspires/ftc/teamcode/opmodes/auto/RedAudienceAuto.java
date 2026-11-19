package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.DriveDiagnostics;
import org.firstinspires.ftc.teamcode.pedro.auto.PathUtility;
import org.firstinspires.ftc.teamcode.pedro.auto.RedAudiencePaths;
import org.firstinspires.ftc.teamcode.subsystems.CompleteShooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * State-machine Autonomous for Red Audience quarter using fixed shooting pose and RPM.
 * Keeps AutoAim disabled; checks Vision for Red Audience scorability, spins flywheel
 * from 800 RPM standby to SHOT_FLYWHEEL_RPM, opens blocker, and feeds POLLEN.
 */
@Autonomous(name = "Red Audience Auto - State Machine", group = "Autonomous")
public class RedAudienceAuto extends OpMode {

    private enum RAAutoState implements PathUtility.AutoState {
        READY,
        INITIAL_APPROACH_Path,
        INITIAL_SHOOT,
        DRIVE_TO_GARDEN_Path,
        COLLECT_GARDEN,
        GARDEN_TO_AUDIENCE_Path,
        WAIT_FOR_AUDIENCE_SCORABLE,
        SHOOT_GARDEN_POLLEN,
        DRIVE_TO_FLOWER_Path,
        COLLECT_FLOWER,
        FLOWER_TO_AUDIENCE_Path,
        CONFIRM_AUDIENCE_SCORABLE,
        SHOOT_FLOWER_POLLEN,
        DRIVE_TO_PARK_Path,
        EMERGENCY_PARK_Path,
        FINISHED,
        FAULT
    }

    /** Set false once fixed RPM/hood values are tuned to enable live intake & shooting. */
    private static final boolean NAVIGATION_ONLY = true;
    private static final String LOG_TAG = "RedAudienceAuto";

    private final Robot robot = Robot.INSTANCE;

    // Fixed shooting constants for shootPose (tune after testing)
    private static final double SHOT_FLYWHEEL_RPM = 2800.0;
    private static final double IDLE_FLYWHEEL_RPM = CompleteShooter.IDLE_FLYWHEEL_RPM; // 800.0
    private static final double HOOD_SHOT_POSITION = 0.50;

    private static boolean ENABLE_FLOWER_CYCLE = true;
    private static final boolean testDrive = true;
    private static final boolean TEST_A = true;

    private static final double AUTO_SECONDS = 30.0;
    private static final double SAFETY_MARGIN_SECONDS = 1.0;
    private static final double LOG_INTERVAL_SECONDS = 0.1;
    private static final double PARK_RESERVE_SECONDS = 5.0;

    // Navigation-only simulation timers
    private static final double TEST_COLLECTION = 2.0;
    private static final double TEST_SHOOT = 1.0;

    // Live hardware timers
    private static final double MAX_COLLECTION_TIME = 3.0;
    private static final double LIVE_FEED_DURATION_SECONDS = 1.6;
    private static final double MAX_LIVE_SHOT_STATE_SECONDS = 4.0;
    private static final double DRIVE_TO_FLOWER_TIME = 2.0;
    private static final double FLOWER_CYCLE = 2 * DRIVE_TO_FLOWER_TIME + TEST_COLLECTION + TEST_SHOOT;

    private static final double MAX_POSITION_JUMP_INCHES = 12.0;
    private static final double MAX_HEADING_JUMP_DEGREES = 100.0;

    private Follower follower;
    private DriveDiagnostics driveDiagnostics;
    private RAAutoState currentState;
    private RedAudiencePaths paths;
    private boolean parkingPathStarted;
    private boolean started;

    // Fixed-shot step tracking: 0 = waiting for flywheel RPM, 1 = opening blocker, 2 = feeding
    private int shotStep = 0;
    private boolean flywheelSpinningForShot = false;

    private Pose previousPose;
    private String faultReason = "";

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime shotStepTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        if (NAVIGATION_ONLY) {
            robot.vision.init(hardwareMap, telemetry);
            robot.vision.setEnableTelemetry(false);
        } else {
            robot.initShooterAndVision(
                    hardwareMap,
                    telemetry,
                    Vision.Cell.Color.RED,
                    Vision.Cell.Location.AUDIENCE,
                    false
            );
            robot.turret.setTargetAngle(0.0);
            robot.hood.setPosition(HOOD_SHOT_POSITION);
            flywheelSpinningForShot = false;
        }
        PathUtility.info(LOG_TAG, "Robot subsystems initialized (NAVIGATION_ONLY=" + NAVIGATION_ONLY + ")");

        paths = new RedAudiencePaths();
        driveDiagnostics = new DriveDiagnostics(hardwareMap,
                "leftFront", "rightFront",
                "leftBack", "rightBack", LOG_TAG);

        if (follower != null) {
            follower.setPose(paths.startPose);
        }

        previousPose = follower.pose();
        currentState = RAAutoState.READY;
        started = false;
        parkingPathStarted = false;

        telemetry.addLine("Red Audience Auto initialized (Fixed Pose Mode)");
        telemetry.addData("Navigation only", NAVIGATION_ONLY);
        telemetry.addData("Start X,Y", "%.1f %.1f", follower.pose().x(), follower.pose().y());
        telemetry.addData("Start heading", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        stateTimer.reset();
        logTimer.reset();
        started = true;
        PathUtility.info(LOG_TAG, "Diagnostic_Test= " + (TEST_A ? "A" : "B"));
        changeState(RAAutoState.INITIAL_APPROACH_Path, "Start Auto");
    }

    private boolean isStationaryState() {
        switch (currentState) {
            case INITIAL_SHOOT:
            case COLLECT_FLOWER:
            case COLLECT_GARDEN:
            case WAIT_FOR_AUDIENCE_SCORABLE:
            case CONFIRM_AUDIENCE_SCORABLE:
            case SHOOT_GARDEN_POLLEN:
            case SHOOT_FLOWER_POLLEN:
                return true;
            default:
                return false;
        }
    }

    private boolean isParkingState() {
        return currentState == RAAutoState.DRIVE_TO_PARK_Path
                || currentState == RAAutoState.EMERGENCY_PARK_Path;
    }

    private void stopForStationaryState(String event) {
        Pose previous = follower.pose();
        if (testDrive) {
            driveDiagnostics.log(autoTimer.seconds(), currentState.name(), event + "Before Stop", follower);
        }
        follower.stop();
        Pose after = follower.pose();
        if (testDrive) {
            driveDiagnostics.log(autoTimer.seconds(), currentState.name(), event + "After Stop", follower);
            driveDiagnostics.logPoseDifference(autoTimer.seconds(), currentState.name(), previous, after);
        }
        PathUtility.logPose(LOG_TAG, event, follower, autoTimer, currentState);
    }

    private void beginPath(String name, Path path, Pose target) {
        PathUtility.info(LOG_TAG, PathUtility.pathStart(autoTimer.seconds(), name, target));
        follower.follow(path);
        if (testDrive) {
            driveDiagnostics.log(autoTimer.seconds(), currentState.name(), name + "Path started", follower);
        }
    }

    private boolean validLocalization() {
        Pose now = follower.pose();
        if (now == null || !Double.isFinite(now.x())
                || !Double.isFinite(now.y()) || !Double.isFinite(now.heading())) return false;
        if (previousPose != null) {
            double distance = Math.hypot(now.x() - previousPose.x(), now.y() - previousPose.y());
            double angle = Math.abs(Math.toDegrees(PathUtility.angleDifference(
                    now.heading(), previousPose.heading())));
            if (distance > MAX_POSITION_JUMP_INCHES || angle > MAX_HEADING_JUMP_DEGREES) {
                RobotLog.ee(LOG_TAG, "LOCALIZATION_JUMP dx=%.2f headingDelta=%.1f", distance, angle);
                return false;
            }
        }
        previousPose = now;
        return true;
    }

    private void fault(String reason) {
        faultReason = reason;
        PathUtility.error(LOG_TAG, "Fault: " + reason);
        stopCollection();
        stopShooting();
        if (follower != null) stopForStationaryState("Fault");
        currentState = RAAutoState.FAULT;
        driveDiagnostics.stopAllMotors();
    }

    private void showTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Auto seconds", "%.2f / %.0f", autoTimer.seconds(), AUTO_SECONDS);
        telemetry.addData("Time Elapsed", "%.2f", stateTimer.seconds());
        telemetry.addData("Remaining", "%.2f", Math.max(0, AUTO_SECONDS - autoTimer.seconds()));
        telemetry.addData("Navigation only", NAVIGATION_ONLY);
        telemetry.addData("Audience Scorable", robot.vision.isRedAudienceScorable());
        if (!NAVIGATION_ONLY) {
            telemetry.addData("Flywheel Target / Actual RPM", "%.0f / %.0f",
                    flywheelSpinningForShot ? SHOT_FLYWHEEL_RPM : IDLE_FLYWHEEL_RPM,
                    robot.flywheel.getFlywheelRPM());
            telemetry.addData("Shot Step", shotStep);
        }
        telemetry.addData("Fault", faultReason);
        if (follower != null) {
            telemetry.addData("Pose", PathUtility.formatPose(follower.pose()));
            telemetry.addData("Path busy", follower.isBusy());
        }
        telemetry.update();
    }

    private boolean mustParkNow() {
        return autoTimer.seconds() >= AUTO_SECONDS - PARK_RESERVE_SECONDS;
    }

    @Override
    public void loop() {
        if (!started || currentState == RAAutoState.FAULT || currentState == RAAutoState.FINISHED) {
            showTelemetry();
            return;
        }

        // Update Vision and CompleteShooter every cycle
        robot.vision.periodic();
        if (!NAVIGATION_ONLY) {
            robot.completeShooter.periodic();
            if (flywheelSpinningForShot) {
                robot.flywheel.setFlywheelRPM(SHOT_FLYWHEEL_RPM);
                robot.flywheel.periodic();
            }
            robot.turret.periodic();
            robot.hood.periodic();
            robot.blocker.periodic();
            robot.transfer.periodic();
            robot.intakeRamp.periodic();
        }

        if (TEST_A || !isStationaryState()) {
            follower.update();
        }

        if (logTimer.seconds() >= LOG_INTERVAL_SECONDS) {
            if (testDrive) {
                String event = TEST_A ? "Test A" : "Test B";
                driveDiagnostics.log(autoTimer.seconds(), currentState.name(), event + "Periodic", follower);
            }
            logTimer.reset();
        }

        if (!validLocalization()) {
            fault("Pose discontinuity / invalid localization");
            showTelemetry();
            return;
        }

        if (autoTimer.seconds() >= AUTO_SECONDS) {
            changeState(RAAutoState.FINISHED, "30 seconds due");
            showTelemetry();
            return;
        }

        boolean protectedActiveShooting = (currentState == RAAutoState.SHOOT_GARDEN_POLLEN
                || currentState == RAAutoState.SHOOT_FLOWER_POLLEN)
                && (NAVIGATION_ONLY || shotStep == 2);

        if (!protectedActiveShooting && !isParkingState() && mustParkNow()) {
            PathUtility.warning(LOG_TAG, PathUtility.deadline(autoTimer.seconds(),
                    AUTO_SECONDS - autoTimer.seconds(), PARK_RESERVE_SECONDS));
            changeState(RAAutoState.EMERGENCY_PARK_Path, "Parking Time Reserved");
            showTelemetry();
            return;
        }

        double elapsed = stateTimer.seconds();
        boolean pathComplete = elapsed >= 0.05 && !follower.isBusy();

        switch (currentState) {
            case INITIAL_APPROACH_Path:
                if (pathComplete) changeState(RAAutoState.INITIAL_SHOOT, "Ready to Shoot");
                break;

            case INITIAL_SHOOT:
                if (shootingFinished()) {
                    stopShooting();
                    changeState(RAAutoState.DRIVE_TO_GARDEN_Path, "Shoot Completed");
                }
                break;

            case DRIVE_TO_GARDEN_Path:
                if (pathComplete) changeState(RAAutoState.COLLECT_GARDEN, "Arrived at Garden");
                break;

            case COLLECT_GARDEN:
                if (collectionFinished()) {
                    stopCollection();
                    changeState(RAAutoState.GARDEN_TO_AUDIENCE_Path, "Garden Collection Completed");
                }
                break;

            case GARDEN_TO_AUDIENCE_Path:
                if (pathComplete) changeState(RAAutoState.WAIT_FOR_AUDIENCE_SCORABLE, "Arrived at Shooting pose");
                break;

            case WAIT_FOR_AUDIENCE_SCORABLE:
                if (isAudienceCellScorable()) {
                    changeState(RAAutoState.SHOOT_GARDEN_POLLEN, "Audience Cell Ready");
                } else if (!hasTimeFor(TEST_SHOOT, PARK_RESERVE_SECONDS)) {
                    changeState(RAAutoState.EMERGENCY_PARK_Path, "Audience Cell Not Ready, go park");
                }
                break;

            case SHOOT_GARDEN_POLLEN:
                if (shootingFinished()) {
                    stopShooting();
                    if (canAttemptFlowerCycle()) {
                        changeState(RAAutoState.DRIVE_TO_FLOWER_Path, "Shooting garden pollen finished");
                    } else {
                        changeState(RAAutoState.EMERGENCY_PARK_Path, "No time for flower collection");
                    }
                }
                break;

            case DRIVE_TO_FLOWER_Path:
                if (pathComplete) changeState(RAAutoState.COLLECT_FLOWER, "Arrived at Flower");
                break;

            case COLLECT_FLOWER:
                if (collectionFinished()) {
                    stopCollection();
                    changeState(RAAutoState.FLOWER_TO_AUDIENCE_Path, "Collection Flower finished");
                }
                break;

            case FLOWER_TO_AUDIENCE_Path:
                if (pathComplete) changeState(RAAutoState.CONFIRM_AUDIENCE_SCORABLE, "Arrived at Audience");
                break;

            case CONFIRM_AUDIENCE_SCORABLE:
                if (isAudienceCellScorable()) {
                    if (hasTimeFor(TEST_SHOOT, PARK_RESERVE_SECONDS)) {
                        changeState(RAAutoState.SHOOT_FLOWER_POLLEN, "Audience Cell Ready for shooting");
                    } else {
                        changeState(RAAutoState.EMERGENCY_PARK_Path, "No time to shoot, emergency park");
                    }
                } else if (!hasTimeFor(TEST_SHOOT, PARK_RESERVE_SECONDS)) {
                    changeState(RAAutoState.EMERGENCY_PARK_Path, "Audience Cell not scorable in time");
                }
                break;

            case SHOOT_FLOWER_POLLEN:
                if (shootingFinished()) {
                    stopShooting();
                    changeState(RAAutoState.DRIVE_TO_PARK_Path, "Finish last shooting, Go Park");
                }
                break;

            case DRIVE_TO_PARK_Path:
            case EMERGENCY_PARK_Path:
                if (parkingPathStarted && pathComplete) {
                    changeState(RAAutoState.FINISHED, "Auto completed");
                }
                break;

            case FINISHED:
                break;
        }

        showTelemetry();
    }

    private void changeState(RAAutoState nextState, String reason) {
        if (currentState == RAAutoState.FINISHED || currentState == RAAutoState.FAULT) return;
        if (isParkingState() && nextState != RAAutoState.FINISHED && nextState != RAAutoState.FAULT) return;

        PathUtility.info(LOG_TAG, PathUtility.stateChange(autoTimer.seconds(), currentState, nextState, reason));

        currentState = nextState;
        stateTimer.reset();
        shotStep = 0;

        if (isStationaryState()) {
            stopForStationaryState("Stationary State Entry");
        }

        switch (nextState) {
            case INITIAL_APPROACH_Path:
                // Pre-spin flywheel during approach
                flywheelSpinningForShot = true;
                beginPath("Initial Approach", paths.initialApproach, paths.shootPose);
                break;

            case INITIAL_SHOOT:
            case SHOOT_FLOWER_POLLEN:
            case SHOOT_GARDEN_POLLEN:
                beginShooting();
                break;

            case DRIVE_TO_GARDEN_Path:
                flywheelSpinningForShot = false;
                beginPath("Drive to Garden", paths.driveToGarden, paths.gardenPose);
                break;

            case COLLECT_FLOWER:
                beginCollectionFlower();
                break;

            case COLLECT_GARDEN:
                beginCollectionGarden();
                break;

            case GARDEN_TO_AUDIENCE_Path:
                flywheelSpinningForShot = true;
                beginPath("Garden to Audience", paths.gardenToAudience, paths.shootPose);
                break;

            case DRIVE_TO_FLOWER_Path:
                flywheelSpinningForShot = false;
                beginPath("Drive to Flower", paths.driveToFlower, paths.flowerPose);
                break;

            case FLOWER_TO_AUDIENCE_Path:
                flywheelSpinningForShot = true;
                beginPath("Flower to Audience", paths.flowerToAudience, paths.shootPose);
                break;

            case EMERGENCY_PARK_Path:
                flywheelSpinningForShot = false;
                stopCollection();
                stopShooting();
                stopForStationaryState("Emergency Park");
                Pose current = follower.pose();
                if (current == null) {
                    fault("No pose for emergency park");
                    return;
                }
                parkingPathStarted = true;
                beginPath("Emergency Park", paths.emergencyPark(current), paths.parkPose);
                break;

            case DRIVE_TO_PARK_Path:
                flywheelSpinningForShot = false;
                stopCollection();
                stopShooting();
                parkingPathStarted = true;
                beginPath("Drive to Park", paths.driveToPark, paths.parkPose);
                break;

            case WAIT_FOR_AUDIENCE_SCORABLE:
            case CONFIRM_AUDIENCE_SCORABLE:
                flywheelSpinningForShot = true;
                PathUtility.info(LOG_TAG, "Waiting for scorable Red Audience cell: " + nextState);
                PathUtility.logPose(LOG_TAG, "Cell Wait start", follower, autoTimer, currentState);
                break;

            case FINISHED:
                flywheelSpinningForShot = false;
                stopCollection();
                stopShooting();
                if (testDrive) {
                    driveDiagnostics.log(autoTimer.seconds(), currentState.name(),
                            "In finished, before calling stop", follower);
                }
                stopForStationaryState("Finished");
                follower.update();
                if (testDrive) {
                    driveDiagnostics.log(autoTimer.seconds(), currentState.name(),
                            "In finished, after calling stop", follower);
                    driveDiagnostics.stopAllMotors();
                }
                break;

            default:
                break;
        }
    }

    private boolean canAttemptFlowerCycle() {
        return ENABLE_FLOWER_CYCLE && hasTimeFor(FLOWER_CYCLE, PARK_RESERVE_SECONDS);
    }

    private void beginShooting() {
        if (NAVIGATION_ONLY) return;
        shotStep = 0;
        flywheelSpinningForShot = true;
        robot.flywheel.setFlywheelRPM(SHOT_FLYWHEEL_RPM);
    }

    private boolean shootingFinished() {
        if (NAVIGATION_ONLY) {
            return stateTimer.seconds() >= TEST_SHOOT;
        }

        if (shotStep == 0) {
            if (robot.flywheel.isReadyToShoot()) {
                robot.blocker.open();
                shotStepTimer.reset();
                shotStep = 1;
            }
        } else if (shotStep == 1) {
            if (robot.blocker.isReady()
                    && shotStepTimer.seconds() >= CompleteShooter.BLOCKER_OPEN_SECONDS) {
                robot.transfer.setFeedPower(CompleteShooter.FEED_POWER);
                shotStepTimer.reset();
                shotStep = 2;
            }
        } else if (shotStep == 2) {
            if (shotStepTimer.seconds() >= LIVE_FEED_DURATION_SECONDS) {
                return true;
            }
            if (!isAudienceCellScorable()) {
                return true;
            }
        }

        return stateTimer.seconds() >= MAX_LIVE_SHOT_STATE_SECONDS;
    }

    private void stopShooting() {
        if (NAVIGATION_ONLY) return;
        flywheelSpinningForShot = false;
        shotStep = 0;
        robot.transfer.stopFeed();
        robot.blocker.close();
    }

    private void beginCollectionGarden() {
        if (NAVIGATION_ONLY) return;
        robot.completeShooter.startCollecting();
    }

    private void beginCollectionFlower() {
        if (NAVIGATION_ONLY) return;
        robot.completeShooter.startCollecting();
    }

    private boolean collectionFinished() {
        if (NAVIGATION_ONLY) {
            return stateTimer.seconds() >= TEST_COLLECTION;
        }
        return stateTimer.seconds() >= MAX_COLLECTION_TIME;
    }

    private void stopCollection() {
        if (NAVIGATION_ONLY) return;
        robot.completeShooter.stopCollecting();
    }

    private boolean isAudienceCellScorable() {
        Vision.Cell cell = robot.vision.getRedAudienceCell();
        if (cell != null && cell.isScorable()) {
            PathUtility.info(LOG_TAG, "Red Audience Cell scorable: " + cell.getNameAndColor());
            return true;
        }
        return false;
    }

    private boolean hasTimeFor(double taskSeconds, double parkSeconds) {
        double remaining = AUTO_SECONDS - autoTimer.seconds();
        return remaining >= taskSeconds + parkSeconds + SAFETY_MARGIN_SECONDS;
    }

    @Override
    public void stop() {
        flywheelSpinningForShot = false;
        if (!NAVIGATION_ONLY) {
            robot.completeShooter.stop();
        }
        if (follower != null) follower.stop();
        if (driveDiagnostics != null) driveDiagnostics.stopAllMotors();
        robot.vision.close();
        PathUtility.info(LOG_TAG, "Auto stopped, camera closed, drivetrain stopped.");
    }
}