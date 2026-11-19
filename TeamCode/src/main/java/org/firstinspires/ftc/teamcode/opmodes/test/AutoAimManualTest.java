package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Tests stationary vision-guided aiming WITHOUT feeding or shooting POLLEN.
 * Extends NextFTCOpMode so subsystem periodic() loops and button triggers are scheduler-managed.
 *
 * Gamepad 1:
 *   A: enable automatic aiming
 *   B: disable aiming (stops turret/flywheel; hood holds position)
 *   X: select RED alliance (disables aiming)
 *   Y: select BLUE alliance (disables aiming)
 *   D-Pad Up: cycle target location (ANY / SCORING / AUDIENCE)
 */
@TeleOp(name = "Auto Aim Manual Test", group = "Test")
public final class AutoAimManualTest extends NextFTCOpMode {
    private final AutoAim aim = AutoAim.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Vision vision = Vision.INSTANCE;

    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private Vision.Cell.Location targetLocation = null; // null = ANY (Scoring or Audience)

    public AutoAimManualTest() {
        addComponents(
                new SubsystemComponent(
                        vision,
                        turret,
                        hood,
                        flywheel,
                        aim
                )
        );
    }

    @Override
    public void onInit() {
        turret.init(hardwareMap.get(DcMotorEx.class, "turret"));
        hood.init(hardwareMap.get(Servo.class, "hood"));
        flywheel.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        vision.init(hardwareMap, telemetry);
        vision.setEnableTelemetry(false);

        aim.init(alliance, targetLocation);
        aim.setCallSubPeriodic(false);
        aim.setAutoCloseBlockerOnStop(false);

        telemetry.addLine("A: enable aim | B: disable aim | D-Pad UP: location");
        telemetry.addLine("X: RED | Y: BLUE (selection disables aim)");
        telemetry.addLine("No transfer motors are commanded by this OpMode.");
        telemetry.addLine("Remove POLLEN and verify turret/hood clearance before testing.");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().a().whenBecomesTrue(aim.enableCommand());
        Gamepads.gamepad1().b().whenBecomesTrue(aim.disableCommand());

        Gamepads.gamepad1().x().whenBecomesTrue(
                new InstantCommand(() -> selectTarget(Vision.Cell.Color.RED, targetLocation))
        );
        Gamepads.gamepad1().y().whenBecomesTrue(
                new InstantCommand(() -> selectTarget(Vision.Cell.Color.BLUE, targetLocation))
        );
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(new InstantCommand(this::cycleLocation));
    }

    @Override
    public void onUpdate() {
        Vision.Cell cell = (targetLocation == null)
                ? vision.getBestCell(alliance)
                : vision.getBestCell(alliance, targetLocation);
        boolean turretReady = aim.isEnabled() && turret.isAtTarget();
        boolean flywheelReady = aim.isEnabled() && flywheel.isReadyToShoot();
        boolean hoodReady = aim.isEnabled() && hood.isReady();

        telemetry.addLine("=== AUTO AIM MANUAL TEST ===");
        telemetry.addData("Target Filter", "%s | Location=%s",
                alliance, targetLocation == null ? "ANY" : targetLocation);
        telemetry.addData("Aim enabled", aim.isEnabled());
        telemetry.addData("Aim status", aim.getStatus());
        telemetry.addData("Shot table verified", AutoAim.CALIBRATION_VERIFIED);
        telemetry.addData("Hood angle calibration verified", hood.hasValidAngleCalibration());
        telemetry.addData("CELL", cell == null ? "none" : cell.getNameAndColor());
        telemetry.addData("CELL scorable", cell != null && cell.isScorable());
        telemetry.addData("Distance (in)", "%.2f", aim.getDistanceInches());

        telemetry.addLine("--- Turret ---");
        telemetry.addData("Target angle (deg)", "%.2f", aim.getTargetTurretDegrees());
        telemetry.addData("Actual angle (deg)", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Turret ready", turretReady);

        telemetry.addLine("--- Hood ---");
        telemetry.addData("Target launch angle (deg)", "%.2f", aim.getTargetHoodAngleDegrees());
        telemetry.addData("Converted servo target", "%.4f", aim.getTargetHoodPosition());
        telemetry.addData("Servo commanded position", "%.4f", hood.getPosition());
        telemetry.addData("Hood ready (TIME ESTIMATE)", hoodReady);
        telemetry.addData("Hood settling seconds", "%.2f", Hood.SETTLE_SECONDS);

        telemetry.addLine("--- Flywheel ---");
        telemetry.addData("Target RPM", "%.1f", aim.getTargetFlywheelRPM());
        telemetry.addData("Actual RPM", "%.1f", flywheel.getFlywheelRPM());
        telemetry.addData("Flywheel ready", flywheelReady);

        telemetry.addLine("--- Combined readiness ---");
        telemetry.addData("Turret / Flywheel / Hood", "%s / %s / %s",
                turretReady, flywheelReady, hoodReady);
        telemetry.addData("AutoAim ready", aim.isReadyToShoot());
        telemetry.addLine("A enable | B disable | X RED | Y BLUE | D-Pad UP location");
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
        aim.disable();
        alliance = color;
        targetLocation = location;
        aim.init(color, location);
        aim.setCallSubPeriodic(false);
    }

    @Override
    public void onStop() {
        aim.disable();
        vision.close();
    }
}