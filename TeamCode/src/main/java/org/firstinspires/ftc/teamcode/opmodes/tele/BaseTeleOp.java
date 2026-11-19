package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Shared NextFTC TeleOp base using dev.nextftc.ftc.Gamepads for all controller inputs.
 * Supports:
 * - LocalizationMode.FIELD (uses Robot.INSTANCE.dt + Robot.INSTANCE.odom Pinpoint heading)
 * - LocalizationMode.PEDRO (uses Pedro Pathing Follower teleop drive &amp; field pose)
 * - 1-Driver or 2-Driver gamepad layouts
 * - Haptic controller rumble + telemetry notification when AutoAim locks onto a scorable cell
 */
public abstract class BaseTeleOp extends NextFTCOpMode {

    public enum LocalizationMode {
        FIELD,
        PEDRO
    }

    protected final Robot robot = Robot.INSTANCE;
    private final LocalizationMode localizationMode;
    private final boolean twoDrivers;

    private Follower pedroFollower;
    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private double headingOffsetRad = 0.0;
    private boolean wasAimReady = false;
    private boolean slowMode = false;

    protected BaseTeleOp(LocalizationMode localizationMode, boolean twoDrivers) {
        this.localizationMode = localizationMode;
        this.twoDrivers = twoDrivers;

        List<Subsystem> subsystems = new ArrayList<>(Arrays.asList(robot.shooterAndVisionSubsystems()));
        if (localizationMode == LocalizationMode.FIELD) {
            subsystems.add(robot.dt);
        }

        addComponents(
                new SubsystemComponent(subsystems.toArray(new Subsystem[0])),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize Vision & Shooter subsystems (location = null tracks any alliance scorable cell)
        robot.initShooterAndVision(hardwareMap, telemetry, alliance, null, true);

        if (localizationMode == LocalizationMode.PEDRO) {
            pedroFollower = Constants.create(hardwareMap);
        } else {
            robot.odom.init(hardwareMap, telemetry);
        }

        wasAimReady = false;
        slowMode = false;

        telemetry.addLine("TeleOp Initialized: " + localizationMode + " | " + (twoDrivers ? "2 Drivers" : "1 Driver"));
        telemetry.addLine("Press D-Pad LEFT for RED, D-Pad RIGHT for BLUE before Start");
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {
        boolean selectRed = Gamepads.gamepad1().dpadLeft().get()
                || (twoDrivers && Gamepads.gamepad2().dpadLeft().get());
        boolean selectBlue = Gamepads.gamepad1().dpadRight().get()
                || (twoDrivers && Gamepads.gamepad2().dpadRight().get());

        if (selectRed && alliance != Vision.Cell.Color.RED) {
            alliance = Vision.Cell.Color.RED;
            robot.completeShooter.init(alliance, null);
        } else if (selectBlue && alliance != Vision.Cell.Color.BLUE) {
            alliance = Vision.Cell.Color.BLUE;
            robot.completeShooter.init(alliance, null);
        }

        if (localizationMode == LocalizationMode.FIELD) {
            robot.odom.periodic();
        } else if (pedroFollower != null) {
            pedroFollower.update();
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Localization", localizationMode);
        telemetry.addData("Drivers", twoDrivers ? "2 Drivers" : "1 Driver");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        robot.completeShooter.enableAim();
        configureBindings();
    }

    private void configureBindings() {
        // --- Driver 1 Drivetrain Controls (Gamepads.gamepad1()) ---
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            if (localizationMode == LocalizationMode.PEDRO && pedroFollower != null) {
                headingOffsetRad = pedroFollower.pose().heading();
            } else {
                headingOffsetRad = robot.odom.getHeadingRadians();
            }
        });

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> slowMode = !slowMode);

        // --- Mechanism Controls (Gamepads.gamepad2() in 2-Driver mode, Gamepads.gamepad1() in 1-Driver mode) ---
        if (twoDrivers) {
            bindMechanismControls(Gamepads.gamepad2());
        } else {
            bindMechanismControls(Gamepads.gamepad1());
        }
    }

    private void bindMechanismControls(GamepadEx pad) {
        // Hold Right Bumper: Lower IntakeRamp + run floor intake; Release: Raise IntakeRamp + stop intake
        pad.rightBumper()
                .whenBecomesTrue(robot.completeShooter.startCollectingCommand())
                .whenBecomesFalse(robot.completeShooter.stopCollectingCommand());

        // A: Enable AutoAim | B: Disable AutoAim (returns flywheel to 800 RPM standby)
        pad.a().whenBecomesTrue(robot.completeShooter.enableAimCommand());
        pad.b().whenBecomesTrue(robot.completeShooter.disableAimCommand());

        // Hold Right Trigger (> 0.3): Request shot (waits for aim lock, opens blocker, feeds POLLEN)
        // Release Right Trigger: Stop feeding & close blocker immediately
        pad.rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(robot.completeShooter.requestShotCommand())
                .whenBecomesFalse(robot.completeShooter.cancelShotCommand());

        // X: Cancel active shot / close blocker
        pad.x().whenBecomesTrue(robot.completeShooter.cancelShotCommand());
    }

    @Override
    public void onUpdate() {
        // Read Driver 1 sticks exclusively through Gamepads.gamepad1()
        double forward = -Gamepads.gamepad1().leftStickY().get();
        double strafe = Gamepads.gamepad1().leftStickX().get();
        double turn = Gamepads.gamepad1().rightStickX().get();

        double scale = slowMode ? 0.45 : 1.0;
        forward *= scale;
        strafe *= scale;
        turn *= scale;

        if (localizationMode == LocalizationMode.PEDRO) {
            updatePedroDrive(forward, strafe, turn);
        } else {
            updateFieldDrive(forward, strafe, turn);
        }

        // Notify driver(s) with haptic rumble on the rising edge of aim lock
        boolean aimReady = robot.completeShooter.isAimReady();
        if (aimReady && !wasAimReady) {
            gamepad1.rumble(250);
            if (twoDrivers) {
                gamepad2.rumble(250);
            }
        }
        wasAimReady = aimReady;

        telemetry.addData("Alliance / Mode", "%s | %s (%s)",
                alliance, localizationMode, twoDrivers ? "2-Driver" : "1-Driver");
        telemetry.addData("AIM STATUS", aimReady
                ? ">>> READY TO SHOOT (" + robot.completeShooter.getTrackedCellName() + ") <<<"
                : "Searching / Aligning...");
        telemetry.addData("Shooter Stage", "%s | %s",
                robot.completeShooter.getStage(), robot.completeShooter.getStatus());
        telemetry.addData("Collecting (Ramp Down)", robot.completeShooter.isCollecting());
        telemetry.addData("Slow Mode", slowMode);

        if (localizationMode == LocalizationMode.PEDRO && pedroFollower != null) {
            Pose p = pedroFollower.pose();
            telemetry.addData("Pedro Pose (X, Y, Deg)", "%.1f, %.1f, %.1f°",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
        } else {
            telemetry.addData("Field Pose (X, Y, Deg)", "%.1f, %.1f, %.1f°",
                    robot.odom.getXInches(), robot.odom.getYInches(), Math.toDegrees(robot.odom.getHeadingRadians()));
        }
        telemetry.update();
    }

    private void updateFieldDrive(double forward, double strafe, double turn) {
        robot.odom.periodic();

        // Hold Left Trigger (> 0.3) for Robot-Centric fine alignment toward the cell;
        // otherwise drive Field-Centric for fast navigation across the field.
        if (Gamepads.gamepad1().leftTrigger().get() > 0.3) {
            robot.dt.drive(forward * 0.5, strafe * 0.5, turn * 0.5);
            return;
        }

        double robotHeading = robot.odom.getHeadingRadians() - headingOffsetRad;
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double rotForward = forward * cos - strafe * sin;
        double rotStrafe = forward * sin + strafe * cos;

        robot.dt.drive(rotForward, rotStrafe, turn);
    }
    private void updatePedroDrive(double forward, double strafe, double turn) {
        if (pedroFollower == null) return;
        // Apply driver heading offset so pressing Y re-zeros field-centric forward in PEDRO mode too
        double cos = Math.cos(headingOffsetRad);
        double sin = Math.sin(headingOffsetRad);
        double adjForward = forward * cos - strafe * sin;
        double adjStrafe = forward * sin + strafe * cos;

        pedroFollower.drivetrain.drive(new DrivePowers(adjForward, -adjStrafe, -turn), false);
        pedroFollower.update();
    }

    @Override
    public void onStop() {
        robot.completeShooter.stop();
        if (localizationMode == LocalizationMode.PEDRO && pedroFollower != null) {
            pedroFollower.stop();
        } else {
            robot.dt.stop();
        }
        robot.vision.close();
    }
}
