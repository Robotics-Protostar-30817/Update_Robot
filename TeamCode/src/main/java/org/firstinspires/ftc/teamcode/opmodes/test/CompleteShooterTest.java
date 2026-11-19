package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.CompleteShooter;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRamp;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Complete shooter integration test extending NextFTCOpMode.
 * Start without POLLEN and use a safe backstop.
 *
 * Gamepad 1:
 *   A: aim ON | B: aim OFF / cancel
 *   X: select RED | Y: select BLUE
 *   D-Pad Up: cycle target location (ANY / SCORING / AUDIENCE)
 *   Left Bumper: toggle floor collection | D-Pad Left: stop collection
 *   Right Bumper: auto-aim AND shoot continuously | D-Pad Down: STOP shooting
 */
@TeleOp(name = "Complete Shooter Integration Test", group = "Test")
public final class CompleteShooterTest extends NextFTCOpMode {
    private final CompleteShooter shooter = CompleteShooter.INSTANCE;
    private final AutoAim aim = AutoAim.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Transfer transfer = Transfer.INSTANCE;
    private final Blocker blocker = Blocker.INSTANCE;
    private final IntakeRamp ramp = IntakeRamp.INSTANCE;
    private final Vision vision = Vision.INSTANCE;

    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private Vision.Cell.Location targetLocation = null; // null = ANY (Scoring or Audience)
    private boolean rumbleSentForRequest;

    public CompleteShooterTest() {
        // NextFTCOpMode includes bindings support by default; register all active subsystems here.
        addComponents(
                new SubsystemComponent(
                        vision,
                        turret,
                        hood,
                        flywheel,
                        transfer,
                        blocker,
                        ramp,
                        aim,
                        shooter
                )
        );
    }

    @Override
    public void onInit() {
        turret.init(hardwareMap.get(DcMotorEx.class, "turret"));
        hood.init(hardwareMap.get(Servo.class, "hood"));
        flywheel.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        transfer.init(
                hardwareMap.get(DcMotorEx.class, "transferMotor1"),
                hardwareMap.get(DcMotorEx.class, "transferMotor2")
        );
        blocker.init(hardwareMap.get(Servo.class, "blocker"));
        ramp.init(hardwareMap.get(Servo.class, "intakeRamp"));
        vision.init(hardwareMap, telemetry);
        vision.setEnableTelemetry(false);

        shooter.init(alliance, targetLocation);
        shooter.setCallSubPeriodic(false);

        telemetry.addLine("A aim ON | B aim OFF | X RED | Y BLUE | D-Pad UP location");
        telemetry.addLine("LB toggle collection | RB auto-aim + continuous shooting");
        telemetry.addLine("D-pad DOWN STOP shooting | LEFT stop collection");
        telemetry.addLine("FIRST TEST WITHOUT POLLEN. Check blocker CLOSED position.");
        telemetry.addLine("STOP robot before shooting: no motion compensation in AutoAim.");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().a().whenBecomesTrue(shooter.enableAimCommand());
        Gamepads.gamepad1().b().whenBecomesTrue(shooter.disableAimCommand());

        Gamepads.gamepad1().x().whenBecomesTrue(
                new InstantCommand(() -> selectTarget(Vision.Cell.Color.RED, targetLocation))
        );
        Gamepads.gamepad1().y().whenBecomesTrue(
                new InstantCommand(() -> selectTarget(Vision.Cell.Color.BLUE, targetLocation))
        );
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(new InstantCommand(this::cycleLocation));

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(new InstantCommand(() -> {
            if (shooter.isCollecting()) shooter.stopCollecting();
            else shooter.startCollecting();
        }));
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(shooter.stopCollectingCommand());

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(new InstantCommand(() -> {
            if (!shooter.isShootingActive()) {
                rumbleSentForRequest = false;
                shooter.requestShot();
            }
        }));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(new InstantCommand(() -> {
            shooter.cancelShot();
            rumbleSentForRequest = false;
        }));
    }

    @Override
    public void onUpdate() {
        boolean aimReadyNow = shooter.isAimReady();
        if (shooter.isShootingActive() && aimReadyNow && !rumbleSentForRequest) {
            gamepad1.rumble(300);
            rumbleSentForRequest = true;
        }

        Vision.Cell cell = (targetLocation == null)
                ? vision.getBestCell(alliance)
                : vision.getBestCell(alliance, targetLocation);

        telemetry.addData("Target Filter", "%s | Location=%s",
                alliance, targetLocation == null ? "ANY" : targetLocation);
        telemetry.addData("Shot calibration verified", AutoAim.CALIBRATION_VERIFIED);
        telemetry.addData("Hood calibration valid", hood.hasValidAngleCalibration());
        telemetry.addData("Aim enabled", aim.isEnabled());
        telemetry.addData("Aim status", aim.getStatus());
        telemetry.addData("Detected CELL", cell == null ? "none" : cell.getNameAndColor());
        telemetry.addData("CELL distance (in)", "%.2f", aim.getDistanceInches());
        telemetry.addData("Turret target (deg)", "%.2f", aim.getTargetTurretDegrees());
        telemetry.addData("Turret measured (deg)", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Turret ready", turret.isAtTarget());
        telemetry.addData("Hood target launch (deg)", "%.2f", aim.getTargetHoodAngleDegrees());
        telemetry.addData("Hood target / cmd pos", "%.4f / %.4f", aim.getTargetHoodPosition(), hood.getPosition());
        telemetry.addData("Hood time ready", hood.isReady());
        telemetry.addData("Flywheel target / actual RPM", "%.0f / %.0f",
                aim.getTargetFlywheelRPM(), flywheel.getFlywheelRPM());
        telemetry.addData("Flywheel speed ready", flywheel.isReadyToShoot());
        telemetry.addData("Aim ready", aimReadyNow);
        telemetry.addData("Collecting", shooter.isCollecting());
        telemetry.addData("Lower intake power", "%.2f", transfer.getIntakePower());
        telemetry.addData("Ramp lowered / pos", "%b / %.3f", ramp.isLowered(), ramp.getPosition());
        telemetry.addData("Upper feed power", "%.2f", transfer.getFeedPower());
        telemetry.addData("Blocker open / ready", "%b / %b", blocker.isOpen(), blocker.isReady());
        telemetry.addData("Shot pending", shooter.isShotPending());
        telemetry.addData("Continuous feeding", shooter.isFiring());
        telemetry.addData("Storage empty detection", "NOT AVAILABLE - operator must check");
        telemetry.addData("Stage", shooter.getStage());
        telemetry.addData("Shooter status", shooter.getStatus());
        telemetry.update();
    }

    private void cycleLocation() {
        Vision.Cell.Location next;
        if (targetLocation == null) next = Vision.Cell.Location.SCORING;
        else if (targetLocation == Vision.Cell.Location.SCORING) next = Vision.Cell.Location.AUDIENCE;
        else next = null;
        selectTarget(alliance, next);
    }

    private void selectTarget(Vision.Cell.Color color, Vision.Cell.Location location) {
        shooter.stop();
        alliance = color;
        targetLocation = location;
        shooter.init(color, location);
        shooter.setCallSubPeriodic(false);
        rumbleSentForRequest = false;
    }

    @Override
    public void onStop() {
        shooter.stop();
        vision.close();
    }
}