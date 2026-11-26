package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.DriveDiagnostics;
import org.firstinspires.ftc.teamcode.pedro.auto.PathUtility;
import org.firstinspires.ftc.teamcode.pedro.auto.RedScoringPaths;
import org.firstinspires.ftc.teamcode.subsystems.CompleteShooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * NextFTC command-based Red Scoring Autonomous using fixed shooting pose and RPM.
 * Keeps AutoAim disabled; checks Vision for Red Scoring scorablity, spins flywheel
 * from 800 RPM standby to SHOT_FLYWHEEL_RPM, opens blocker, and feeds POLLEN.
 */
@Autonomous(name = "RedScoring - NextFTC testing Navigation Only", group = "Autonomous")
public class RedScoringNextFTCAuto_NavigationOnly  extends NextFTCOpMode {

    private static final String LOG_TAG = "RedScoringNextFTCNavigation";
    private static final double AUTO_SECONDS = 30.0;
    private static final double PARK_RESERVE_SECONDS = 3.5;

    // Fixed shooting constants for scoringPose (tune after testing)
    private static final double SHOT_FLYWHEEL_RPM = 2800.0;
    private static final double IDLE_FLYWHEEL_RPM = CompleteShooter.IDLE_FLYWHEEL_RPM; // 800.0
    private static final double HOOD_SHOT_POSITION = 0.50;
    private static final double FEED_DURATION_SECONDS = 1.6;
    private static final double COLLECT_SETTLE_SECONDS = 2.0;

    private static final double CELL_SCORABLE_SECONDS = 5.0;
    //private final Robot robot = Robot.INSTANCE;

    private final Vision vision = Vision.INSTANCE;

    private Follower follower;
    private RedScoringPaths paths;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private boolean emergencyParkTriggered = false;
    private boolean normalParkingStarted = false;
    private boolean flywheelSpinningForShot = false;
    private String phaseStatus = "Initialized";

    private final ElapsedTime logTimer = new ElapsedTime();

    private DriveDiagnostics driveDiagnostics;

    public RedScoringNextFTCAuto_NavigationOnly() {
      //  addComponents(new SubsystemComponent(robot.shooterAndVisionSubsystems()));
        addComponents(new SubsystemComponent(vision));
    }

    @Override
    public void onInit() {
        follower = Constants.create(hardwareMap);
        paths = new RedScoringPaths();
        follower.setPose(paths.startPose);
        follower.update();
        // Matches Constants.drivetrainConfig motor names without touching motor settings
        driveDiagnostics = new DriveDiagnostics(
                hardwareMap,
                "leftFront",
                "rightFront",
                "leftBack",
                "rightBack",
                LOG_TAG
        );
        vision.init(hardwareMap, telemetry);
        /*
        robot.initShooterAndVision(
                hardwareMap,
                telemetry,
                Vision.Cell.Color.RED,
                Vision.Cell.Location.SCORING,
                true
        );

        // Lock turret at center and set fixed hood angle for scoringPose
        robot.turret.setTargetAngle(0.0);
        robot.hood.setPosition(HOOD_SHOT_POSITION);
        flywheelSpinningForShot = false;
        */
        telemetry.addLine("Red Scoring NextFTC Navigation Only  (Fixed Pose Mode)");
        telemetry.update();
    }


    @Override
    public void onStartButtonPressed() {
        autoTimer.reset();
        emergencyParkTriggered = false;
        normalParkingStarted = false;

        driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Auto Started", follower);


        new SequentialGroup(
                // 1. Approach scoringPose while pre-spinning flywheel to SHOT_FLYWHEEL_RPM
                new InstantCommand(() -> phaseStatus = "1. Approach + Spin Up"),
                new ParallelGroup(
                        followPathCommand(paths.initialApproach)
              //          spinUpFlywheelCommand()
                ),
               //new Delay(CELL_SCORABLE_SECONDS),
                new InstantCommand(()->phaseStatus="1.5, wait for scoring cell to be scorable"),
                waitForScoringThenShootCommand(),
                //waitForScoringScorableCommand(CELL_SCORABLE_SECONDS),
            //    shootFixedVolleyCommand("1. Shoot Preload"),

                // 2. Drive to Flower & collect in parallel (drop flywheel to 800 RPM standby)
                new InstantCommand(() -> phaseStatus = "2. Drive & Collect Flower"),
             //   idleFlywheelCommand(),
                new ParallelGroup(
                        followPathCommand(paths.driveToFlower)
                     //   robot.completeShooter.startCollectingCommand()
                ),
                new Delay(COLLECT_SETTLE_SECONDS),

                // 3. Return to scoringPose & pre-spin flywheel during transit
                new InstantCommand(() -> phaseStatus = "3. Return + Spin Up"),
                new ParallelGroup(
                        followPathCommand(paths.flowerToScoring)
                      //  new SequentialGroup(
                     //           new Delay(0.5),
                       //         robot.completeShooter.stopCollectingCommand(),
                       //         spinUpFlywheelCommand()
                      //  )
                ),
                new Delay(CELL_SCORABLE_SECONDS),
                //waitForScoringScorableCommand(2.5),
              //  shootFixedVolleyCommand("3. Shoot Flower Pollen"),

                // 4. Park
                new InstantCommand(() -> {
                    phaseStatus = "4. Parking";
                    normalParkingStarted = true;
                }),
               // new InstantCommand(() -> phaseStatus = "4. Parking"),
              //  idleFlywheelCommand(),
                followPathCommand(paths.scoringToPark),
                new InstantCommand(() -> phaseStatus = "Finished")
        ).schedule();
    }

   // @Override
   // public void onStartButtonPressed() {
   //     autoTimer.reset();
   //     phaseStatus = "STARTED - NO MOVEMENT";
   // }
/*
    @Override
    public void onUpdate() {
        follower.update();

        telemetry.addData("Status", phaseStatus);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );
        telemetry.update();
    }
    */
    @Override
    public void onUpdate() {
        follower.update();

        // Periodic logging every 0.10s (matching RedScoringAuto)
        if (logTimer.seconds() >= 0.10) {
            driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Periodic", follower);
            logTimer.reset();
        }


        // Keep commanding target RPM while shooting/spinning up so CompleteShooter's
        // 800 RPM idle loop does not override it
       // if (flywheelSpinningForShot) {
       //     robot.flywheel.setFlywheelRPM(SHOT_FLYWHEEL_RPM);
       // }

        if (!emergencyParkTriggered && !normalParkingStarted  &&
                autoTimer.seconds() >= AUTO_SECONDS - PARK_RESERVE_SECONDS) {
            emergencyParkTriggered = true;
           // flywheelSpinningForShot = false;
            phaseStatus = "EMERGENCY PARK";
            PathUtility.warning(LOG_TAG, "Triggering emergency park at t=" + autoTimer.seconds());
            driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Emergency Park Triggered", follower);

            CommandManager.INSTANCE.cancelAll();
          //  robot.transfer.stopFeed();
          //  robot.blocker.close();
          //  robot.completeShooter.stopCollecting();

            Pose current = follower.pose();
            if (current != null) {
                followPathCommand(paths.emergencyPark(current)).schedule();
            }
        }

        telemetry.addData("Phase", phaseStatus);
        telemetry.addData("Time", "%.2f / %.0f s", autoTimer.seconds(), AUTO_SECONDS);
        telemetry.addData("Pose", PathUtility.formatPose(follower.pose()));
       // telemetry.addData("Scoring Scorable", isRedScoringScorable());
       // telemetry.addData("Flywheel Target / Actual RPM", "%.0f / %.0f",
      //          flywheelSpinningForShot ? SHOT_FLYWHEEL_RPM : IDLE_FLYWHEEL_RPM,
      //          robot.flywheel.getFlywheelRPM());
      //  telemetry.addData("Flywheel Ready", robot.flywheel.isReadyToShoot());
        telemetry.update();
    }

    @Override
    public void onStop() {
      //  flywheelSpinningForShot = false;
      //  robot.completeShooter.stop();
        if (follower != null) follower.stop();
        if (driveDiagnostics != null) {
            driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Auto Stopped", follower);
            driveDiagnostics.stopAllMotors();
        }
        vision.close();
        PathUtility.info(LOG_TAG, "Auto stopped, camera closed, drivetrain stopped.");

        //  robot.vision.close();
    }

    /*
    private Command waitForScoringCommand(){
        Command waitForScoring = new IfElseCommand(
                ()->vision.getBestCell(Vision.Cell.Color.RED, Vision.Cell.Location.SCORING)!= null &&
                        autoTimer.seconds() < CELL_SCORABLE_SECONDS,
                new NullCommand() //placeholder for shooting

        );
        return waitForScoring;
    }*/
    /*
    private Command spinUpFlywheelCommand() {
        return new InstantCommand(() -> {
            flywheelSpinningForShot = true;
            robot.flywheel.setFlywheelRPM(SHOT_FLYWHEEL_RPM);
        });
    }
*/
    /*
    private Command idleFlywheelCommand() {
        return new InstantCommand(() -> {
            flywheelSpinningForShot = false;
            robot.flywheel.setFlywheelRPM(IDLE_FLYWHEEL_RPM);
        });
    }
*/
    private Command followPathCommand(Path path) {
        ElapsedTime timer = new ElapsedTime();
        return new LambdaCommand()
                .setStart(() -> {
                    timer.reset();
                    driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Path Started", follower);
                    follower.follow(path);
                })
                .setIsDone(() -> timer.seconds() >= 0.05 && !follower.isBusy())
                .setStop(interrupted -> {
                    Pose before = follower.pose();
                    if (interrupted) follower.stop();
                    Pose after = follower.pose();
                    driveDiagnostics.log(autoTimer.seconds(), phaseStatus,
                            interrupted ? "Path Interrupted" : "Path Finished", follower);
                    driveDiagnostics.logPoseDifference(autoTimer.seconds(), phaseStatus, before, after);

                });
    }

    /*
    private boolean isRedScoringScorable() {
        //Vision.Cell cell = robot.vision.getBestCell(Vision.Cell.Color.RED, Vision.Cell.Location.SCORING);
        Vision.Cell cell = vision.getBestCell(Vision.Cell.Color.RED, Vision.Cell.Location.SCORING);
        return cell != null && cell.isScorable();
    }

    private Command waitForScoringScorableCommand(double maxWaitSeconds) {
        ElapsedTime timer = new ElapsedTime();
        return new LambdaCommand()
                .setStart(timer::reset)
                .setIsDone(() -> isRedScoringScorable() || timer.seconds() >= maxWaitSeconds);
    }*/
    private Command waitForScoringThenShootCommand() {
        ElapsedTime waitTimer = new ElapsedTime();
        final boolean[] cellFound = {false};

        // 1. Wait until a Red cell is in SCORING or the timeout expires
        Command waitForCell = new LambdaCommand()
                .setStart(() -> {
                    waitTimer.reset();
                    cellFound[0] = false;
                    driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Vision Wait Started", follower);
                })
                .setIsDone(() -> {
                    if (vision.getBestCell(Vision.Cell.Color.RED, Vision.Cell.Location.SCORING) != null) {
                        cellFound[0] = true;
                        return true;
                    }
                    return waitTimer.seconds() >= CELL_SCORABLE_SECONDS;
                })
                .setStop(interrupted -> {
                    String event = interrupted
                            ? "Vision Wait Interrupted"
                            : (cellFound[0]
                               ? String.format("Vision Cell Detected (%.2fs)", waitTimer.seconds())
                               : String.format("Vision Wait Timed Out (%.2fs)", waitTimer.seconds()));
                    driveDiagnostics.log(autoTimer.seconds(), phaseStatus, event, follower);
                });

        // 2. Branch using the cached detection result
        Command shootOrSkip = new IfElseCommand(
                () -> cellFound[0],
                new SequentialGroup(
                        new InstantCommand(() -> driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Shooting Preload (Placeholder)", follower)),
                        new NullCommand() // Replace with shootFixedVolleyCommand("1. Shoot Preload") later
                ),
                new InstantCommand(() -> driveDiagnostics.log(autoTimer.seconds(), phaseStatus, "Skipping Shot - No Cell", follower))
        );

        return new SequentialGroup(waitForCell, shootOrSkip);
    }



    /**
     * Waits for flywheel to reach SHOT_FLYWHEEL_RPM, opens blocker, feeds for FEED_DURATION_SECONDS,
     * then closes blocker and stops the upper feed motor.
     */
    /*
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
     */
}