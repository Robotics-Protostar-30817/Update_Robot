package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Two-driver robot-centric TeleOp.
 * Pedro owns the Mecanum motors and Pinpoint localization.
 * NextFTC owns vision, intake, aiming, and shooter commands.
 *
 * This OpMode does not initialize Robot.dt or Robot.odom.
 */
@TeleOp(name = "Pedro TeleOp - Full (Robot Centric)", group = "TeleOp")
public class PedroTeleOpFull extends NextFTCOpMode {

    private static final double DEADZONE = 0.05;
    private static final double SLOW_SCALE = 0.45;

    private final Robot robot = Robot.INSTANCE;
    private Follower follower;
    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private boolean slowMode = false;
    private boolean wasAimReady = false;
    private boolean restoredAutoPose = false;

    public PedroTeleOpFull() {
        addComponents(
                new SubsystemComponent(robot.shooterAndVisionSubsystems()),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        robot.initShooterAndVision(hardwareMap, telemetry, alliance, null, true);
        follower = Constants.create(hardwareMap);

        // Each OpMode creates a new Follower. Restore the last Autonomous field pose
        // after constructing it, before any pre-start localization updates.
        Pose autoEndPose = robot.getAutonomousEndPose();
        restoredAutoPose = autoEndPose != null;
        if (restoredAutoPose) {
            follower.setPose(autoEndPose);
        }
        slowMode = false;
        wasAimReady = false;

        telemetry.addLine("Pedro robot-centric drive + NextFTC shooter ready");
        telemetry.addData("Auto pose restored", restoredAutoPose ? "YES" : "NO - fresh localizer");
        telemetry.addLine("Gamepad 1: left stick move, right stick turn, LB slow");
        telemetry.addLine("Gamepad 2: RB intake, A/B aim on/off, RT shoot, X cancel");
        telemetry.addLine("Before start: D-pad LEFT red / RIGHT blue (either pad)");
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {
        boolean red = Gamepads.gamepad1().dpadLeft().get()
                || Gamepads.gamepad2().dpadLeft().get();
        boolean blue = Gamepads.gamepad1().dpadRight().get()
                || Gamepads.gamepad2().dpadRight().get();

        if (red && !blue && alliance != Vision.Cell.Color.RED) {
            alliance = Vision.Cell.Color.RED;
            robot.completeShooter.init(alliance, null);
        } else if (blue && !red && alliance != Vision.Cell.Color.BLUE) {
            alliance = Vision.Cell.Color.BLUE;
            robot.completeShooter.init(alliance, null);
        }

        if (follower != null) follower.update();
        telemetry.addData("Auto pose restored", restoredAutoPose);
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("Drive: ROBOT CENTRIC");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Preserve BaseTeleOp's initial aiming behavior.
        robot.completeShooter.enableAim();

        // Driver 1: optional precision driving.
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> slowMode = !slowMode);

        // Operator 2: hold to collect; release to stop.
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(robot.completeShooter.startCollectingCommand())
                .whenBecomesFalse(robot.completeShooter.stopCollectingCommand());

        // Operator 2: enable/disable AutoAim.
        Gamepads.gamepad2().a()
                .whenBecomesTrue(robot.completeShooter.enableAimCommand());
        Gamepads.gamepad2().b()
                .whenBecomesTrue(robot.completeShooter.disableAimCommand());

        // Operator 2: hold trigger to request a shot; release cancels feeding.
        Gamepads.gamepad2().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(robot.completeShooter.requestShotCommand())
                .whenBecomesFalse(robot.completeShooter.cancelShotCommand());
        Gamepads.gamepad2().x()
                .whenBecomesTrue(robot.completeShooter.cancelShotCommand());
    }

    @Override
    public void onUpdate() {
        double scale = slowMode ? SLOW_SCALE : 1.0;
        double forward = applyDeadzone(-Gamepads.gamepad1().leftStickY().get()) * scale;
        double strafe = applyDeadzone(Gamepads.gamepad1().leftStickX().get()) * scale;
        double turn = applyDeadzone(Gamepads.gamepad1().rightStickX().get()) * scale;

        // Pedro updates Pinpoint localization; the manual command owns motor power.
        // Signs are inherited from the uploaded BaseTeleOp PEDRO branch.
        follower.update();
        follower.drivetrain.drive(new DrivePowers(forward, -strafe, -turn), false);

        boolean aimReady = robot.completeShooter.isAimReady();
        if (aimReady && !wasAimReady) {
            gamepad1.rumble(250);
            gamepad2.rumble(250);
        }
        wasAimReady = aimReady;

        telemetry.addData("Alliance / Drive", "%s / Robot-centric Pedro", alliance);
        telemetry.addData("Auto pose restored", restoredAutoPose);
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Forward / Strafe / Turn", "%.2f / %.2f / %.2f",
                forward, strafe, turn);
        telemetry.addData("AIM STATUS", aimReady
                ? ">>> READY TO SHOOT (" + robot.completeShooter.getTrackedCellName() + ") <<<"
                : "Searching / Aligning...");
        telemetry.addData("Shooter Stage", "%s | %s",
                robot.completeShooter.getStage(), robot.completeShooter.getStatus());
        telemetry.addData("Collecting", robot.completeShooter.isCollecting());

        Pose pose = follower.pose();
        if (pose != null) {
            telemetry.addData("Pedro Pose (X, Y, Deg)", "%.1f, %.1f, %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
        telemetry.update();
    }

    private static double applyDeadzone(double input) {
        return Math.abs(input) < DEADZONE ? 0.0 : input;
    }

    @Override
    public void onStop() {
        robot.completeShooter.stop();
        if (follower != null) {
            follower.drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
            follower.stop();
        }
        robot.vision.close();
    }
}
