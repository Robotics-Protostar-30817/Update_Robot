package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.PathUtility;
import org.firstinspires.ftc.teamcode.pedro.auto.RedAudiencePaths;
import org.firstinspires.ftc.teamcode.subsystems.CompleteShooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * NextFTC command-based Red Audience Autonomous using fixed shooting pose and RPM.
 * Keeps AutoAim disabled; checks Vision for Red Audience scorability, spins flywheel
 * from 800 RPM standby to SHOT_FLYWHEEL_RPM, opens blocker, and feeds POLLEN.
 */
@Autonomous(name = "Red Audience Auto - NextFTC", group = "Autonomous")
public class RedAudienceNextFTCAuto extends NextFTCOpMode {

    private static final String LOG_TAG = "RedAudienceNextFTCAuto";
    private static final double AUTO_SECONDS = 30.0;
    private static final double PARK_RESERVE_SECONDS = 4.5;

    // Fixed shooting constants for shootPose (tune after testing)
    private static final double SHOT_FLYWHEEL_RPM = 2800.0;//tune robot
    private static final double IDLE_FLYWHEEL_RPM = CompleteShooter.IDLE_FLYWHEEL_RPM; // 800.0
    private static final double HOOD_SHOT_POSITION = 0.50; //tune robot
    private static final double FEED_DURATION_SECONDS = 1.6;
    private static final double COLLECT_SETTLE_SECONDS = 1.5;

    private final Robot robot = Robot.INSTANCE;

    private Follower follower;
    private RedAudiencePaths paths;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private boolean emergencyParkTriggered = false;
    private boolean flywheelSpinningForShot = false;
    private String phaseStatus = "Initialized";

    public RedAudienceNextFTCAuto() {
        addComponents(new SubsystemComponent(robot.shooterAndVisionSubsystems()));
    }

    @Override
    public void onInit() {
        // Invalidate any previous run before starting this Autonomous.
        robot.clearAutonomousEndPose();
        follower = Constants.create(hardwareMap);
        paths = new RedAudiencePaths();
        follower.setPose(paths.startPose);

        robot.initShooterAndVision(
                hardwareMap,
                telemetry,
                Vision.Cell.Color.RED,
                Vision.Cell.Location.AUDIENCE,
                true
        );

        // Lock turret at center and set fixed hood angle for shootPose
        robot.turret.setTargetAngle(0.0);
        robot.hood.setPosition(HOOD_SHOT_POSITION);
        flywheelSpinningForShot = false;

        telemetry.addLine("Red Audience NextFTC Auto Ready (Fixed Pose Mode)");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autoTimer.reset();
        emergencyParkTriggered = false;

        new SequentialGroup(
                // 1. Approach shootPose while pre-spinning flywheel to SHOT_FLYWHEEL_RPM
                new InstantCommand(() -> phaseStatus = "1. Approach + Spin Up"),
                new ParallelGroup(
                        followPathCommand(paths.initialApproach),
                        spinUpFlywheelCommand()
                ),
                waitForAudienceScorableCommand(2.5),
                shootFixedVolleyCommand("1. Shoot Preload"),

                // 2. Drive to Garden & collect (drop flywheel back to 800 RPM standby)
                new InstantCommand(() -> phaseStatus = "2. Drive & Collect Garden"),
                idleFlywheelCommand(),
                new ParallelGroup(
                        followPathCommand(paths.driveToGarden),
                        robot.completeShooter.startCollectingCommand()
                ),
                new Delay(COLLECT_SETTLE_SECONDS),

                // 3. Return to Audience shootPose & pre-spin flywheel during transit
                new InstantCommand(() -> phaseStatus = "3. Return + Spin Up"),
                new ParallelGroup(
                        followPathCommand(paths.gardenToAudience),
                        new SequentialGroup(
                                new Delay(0.5),
                                robot.completeShooter.stopCollectingCommand(),
                                spinUpFlywheelCommand()
                        )
                ),
                waitForAudienceScorableCommand(2.5),
                shootFixedVolleyCommand("3. Shoot Garden Pollen"),

                // 4. Drive to Flower & collect
                new InstantCommand(() -> phaseStatus = "4. Drive & Collect Flower"),
                idleFlywheelCommand(),
                new ParallelGroup(
                        followPathCommand(paths.driveToFlower),
                        robot.completeShooter.startCollectingCommand()
                ),
                new Delay(COLLECT_SETTLE_SECONDS),

                // 5. Return to Audience shootPose & pre-spin flywheel during transit
                new InstantCommand(() -> phaseStatus = "5. Return + Spin Up"),
                new ParallelGroup(
                        followPathCommand(paths.flowerToAudience),
                        new SequentialGroup(
                                new Delay(0.5),
                                robot.completeShooter.stopCollectingCommand(),
                                spinUpFlywheelCommand()
                        )
                ),
                waitForAudienceScorableCommand(2.5),
                shootFixedVolleyCommand("5. Shoot Flower Pollen"),

                // 6. Park
                new InstantCommand(() -> phaseStatus = "6. Parking"),
                idleFlywheelCommand(),
                followPathCommand(paths.driveToPark),
                new InstantCommand(() -> {
                    phaseStatus = "Finished";
                    saveCurrentPose();
                })
        ).schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();
        // Keep the latest valid field pose available for the Auto-to-TeleOp handoff.
        // This also covers early stops, interruptions, and emergency parking.
        saveCurrentPose();

        // Keep commanding target RPM while shooting/spinning up so CompleteShooter's
        // 800 RPM idle loop does not override it
        if (flywheelSpinningForShot) {
            robot.flywheel.setFlywheelRPM(SHOT_FLYWHEEL_RPM);
        }

        // Emergency park watchdog before 30s expiration
        if (!emergencyParkTriggered && autoTimer.seconds() >= AUTO_SECONDS - PARK_RESERVE_SECONDS) {
            emergencyParkTriggered = true;
            flywheelSpinningForShot = false;
            phaseStatus = "EMERGENCY PARK";
            PathUtility.warning(LOG_TAG, "Triggering emergency park at t=" + autoTimer.seconds());
            CommandManager.INSTANCE.cancelAll();
            robot.transfer.stopFeed();
            robot.blocker.close();
            robot.completeShooter.stopCollecting();

            Pose current = follower.pose();
            if (current != null) {
                followPathCommand(paths.emergencyPark(current)).schedule();
            }
        }

        telemetry.addData("Phase", phaseStatus);
        telemetry.addData("Time", "%.2f / %.0f s", autoTimer.seconds(), AUTO_SECONDS);
        telemetry.addData("Pose", PathUtility.formatPose(follower.pose()));
        telemetry.addData("Pose saved for TeleOp", robot.getAutonomousEndPose() != null);
        telemetry.addData("Audience Scorable", robot.vision.isRedAudienceScorable());
        telemetry.addData("Flywheel Target / Actual RPM", "%.0f / %.0f",
                flywheelSpinningForShot ? SHOT_FLYWHEEL_RPM : IDLE_FLYWHEEL_RPM,
                robot.flywheel.getFlywheelRPM());
        telemetry.addData("Flywheel Ready", robot.flywheel.isReadyToShoot());
        telemetry.update();
    }

    @Override
    public void onStop() {
        // Save BEFORE stopping the follower or shutting down hardware.
        saveCurrentPose();
        flywheelSpinningForShot = false;
        robot.completeShooter.stop();
        if (follower != null) follower.stop();
        robot.vision.close();
    }

    /** Saves a defensive copy of the latest Pedro pose in Robot.INSTANCE. */
    private void saveCurrentPose() {
        if (follower != null) {
            robot.saveAutonomousEndPose(follower.pose());
        }
    }

    private Command spinUpFlywheelCommand() {
        return new InstantCommand(() -> {
            flywheelSpinningForShot = true;
            robot.flywheel.setFlywheelRPM(SHOT_FLYWHEEL_RPM);
        });
    }

    private Command idleFlywheelCommand() {
        return new InstantCommand(() -> {
            flywheelSpinningForShot = false;
            robot.flywheel.setFlywheelRPM(IDLE_FLYWHEEL_RPM);
        });
    }

    private Command followPathCommand(Path path) {
        ElapsedTime timer = new ElapsedTime();
        return new LambdaCommand()
                .setStart(() -> {
                    timer.reset();
                    follower.follow(path);
                })
                .setIsDone(() -> timer.seconds() >= 0.05 && !follower.isBusy())
                .setStop(interrupted -> {
                    if (interrupted) follower.stop();
                });
    }

    private Command waitForAudienceScorableCommand(double maxWaitSeconds) {
        ElapsedTime timer = new ElapsedTime();
        return new LambdaCommand()
                .setStart(timer::reset)
                .setIsDone(() -> robot.vision.isRedAudienceScorable() || timer.seconds() >= maxWaitSeconds);
    }

    /**
     * Waits for flywheel to reach SHOT_FLYWHEEL_RPM, opens blocker, feeds for FEED_DURATION_SECONDS,
     * then closes blocker and stops the upper feed motor.
     */
    private Command shootFixedVolleyCommand(String label) {
        ElapsedTime stateTimer = new ElapsedTime();
        ElapsedTime stageTimer = new ElapsedTime();
        final int[] step = {0}; // 0 = wait for flywheel RPM, 1 = wait for blocker open, 2 = feeding

        return new LambdaCommand()
                .setStart(() -> {
                    phaseStatus = label;
                    stateTimer.reset();
                    step[0] = 0;
                    follower.stop();
                    flywheelSpinningForShot = true;
                    robot.flywheel.setFlywheelRPM(SHOT_FLYWHEEL_RPM);
                })
                .setUpdate(() -> {
                    if (step[0] == 0) {
                        if (robot.flywheel.isReadyToShoot()) {
                            robot.blocker.open();
                            stageTimer.reset();
                            step[0] = 1;
                        }
                    } else if (step[0] == 1) {
                        if (robot.blocker.isReady()
                                && stageTimer.seconds() >= CompleteShooter.BLOCKER_OPEN_SECONDS) {
                            robot.transfer.setFeedPower(CompleteShooter.FEED_POWER);
                            stageTimer.reset();
                            step[0] = 2;
                        }
                    }
                })
                .setIsDone(() -> (step[0] == 2 && stageTimer.seconds() >= FEED_DURATION_SECONDS)
                        || stateTimer.seconds() >= 4.0)
                .setStop(interrupted -> {
                    robot.transfer.stopFeed();
                    robot.blocker.close();
                });
    }
}