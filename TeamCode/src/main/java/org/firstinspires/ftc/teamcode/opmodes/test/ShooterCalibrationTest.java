package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRamp;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.Locale;

/**
 * Unified bench and field calibration TeleOp for all 7 shooter/intake actuators:
 * 1. Flywheel (6000-RPM motor)
 * 2. Turret (1150/435-RPM motor)
 * 3. Hood (Speed/Super Speed servo)
 * 4. Blocker Gate (Torque/Speed servo)
 * 5. Intake Motor (1620-RPM motor)
 * 6. Transfer Feed Motor (1620-RPM motor)
 * 7. Intake Ramp (Torque/Speed servo)
 *
 * Controls (Gamepad 1):
 * - Left / Right Bumper: Switch active actuator mode
 * - D-Pad Up / Down: Increase / Decrease target RPM, angle, or servo position
 * - D-Pad Left / Right: Fine step adjustment (or jog turret)
 * - Button A: Enable / Preset 1 (Spin flywheel / Open blocker / Lower ramp / Run intake)
 * - Button B: Stop / Preset 2 (Stop motors / Close blocker / Raise ramp / Turret 0 deg)
 * - Button Y: Zero turret encoder (in Turret mode) or Reverse motor direction (Intake/Feed)
 * - Button X: Record current calibrated constants to Driver Station and RobotLog ("ShooterCal")
 */
@TeleOp(name = "Shooter & Intake Calibration", group = "Calibration")
public class ShooterCalibrationTest extends LinearOpMode {

    private static final String LOG_TAG = "ShooterCal";

    private enum Mode {
        FLYWHEEL("1/7: FLYWHEEL (6000 RPM)"),
        TURRET("2/7: TURRET (1150/435 RPM)"),
        HOOD("3/7: HOOD SERVO"),
        BLOCKER("4/7: BLOCKER GATE SERVO"),
        INTAKE_MOTOR("5/7: FLOOR INTAKE (1620 RPM)"),
        FEED_MOTOR("6/7: TRANSFER FEED (1620 RPM)"),
        INTAKE_RAMP("7/7: INTAKE RAMP SERVO");

        final String label;
        Mode(String label) { this.label = label; }
    }

    private Mode currentMode = Mode.FLYWHEEL;

    // Live test setpoints
    private double testFlywheelRpm = 3000.0;
    private double testTurretTargetDeg = 0.0;
    private double testIntakePower = 0.80;
    private double testFeedPower = 0.80;

    // Edge-detection flags
    private boolean prevLb, prevRb, prevUp, prevDown, prevLeft, prevRight, prevA, prevB, prevX, prevY;
    private String lastSavedBanner = "None (Press X to record constants)";

    @Override
    public void runOpMode() {
        // Initialize all subsystems (verify hardware map names match your Robot Configuration)
        Flywheel.INSTANCE.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        Turret.INSTANCE.init(hardwareMap.get(DcMotorEx.class, "turret"));
        Hood.INSTANCE.init(hardwareMap.get(Servo.class, "hood"));
        Blocker.INSTANCE.init(hardwareMap.get(Servo.class, "blocker"));
        Transfer.INSTANCE.init(
                hardwareMap.get(DcMotorEx.class, "intake"),
                hardwareMap.get(DcMotorEx.class, "feed")
        );
        IntakeRamp.INSTANCE.init(hardwareMap.get(Servo.class, "intakeRamp"));

        // Allow full 0.0 - 1.0 bench calibration range on Hood (be careful near mechanical stops!)
        Hood.MIN_POSITION = 0.0;
        Hood.MAX_POSITION = 1.0;

        telemetry.addLine("Shooter & Intake Calibration Ready.");
        telemetry.addLine("Use LB/RB to switch actuator. Keep hands clear of flywheel!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean lbPressed = gamepad1.left_bumper && !prevLb;
            boolean rbPressed = gamepad1.right_bumper && !prevRb;
            boolean upPressed = gamepad1.dpad_up && !prevUp;
            boolean downPressed = gamepad1.dpad_down && !prevDown;
            boolean leftPressed = gamepad1.dpad_left && !prevLeft;
            boolean rightPressed = gamepad1.dpad_right && !prevRight;
            boolean aPressed = gamepad1.a && !prevA;
            boolean bPressed = gamepad1.b && !prevB;
            boolean xPressed = gamepad1.x && !prevX;
            boolean yPressed = gamepad1.y && !prevY;

            // Switch active calibration mode
            Mode[] modes = Mode.values();
            if (rbPressed) {
                currentMode = modes[(currentMode.ordinal() + 1) % modes.length];
            } else if (lbPressed) {
                currentMode = modes[(currentMode.ordinal() - 1 + modes.length) % modes.length];
            }

            switch (currentMode) {
                case FLYWHEEL:
                    if (upPressed) testFlywheelRpm = Math.min(6000.0, testFlywheelRpm + 250.0);
                    if (downPressed) testFlywheelRpm = Math.max(0.0, testFlywheelRpm - 250.0);
                    if (rightPressed) testFlywheelRpm = Math.min(6000.0, testFlywheelRpm + 50.0);
                    if (leftPressed) testFlywheelRpm = Math.max(0.0, testFlywheelRpm - 50.0);
                    if (aPressed) Flywheel.INSTANCE.setFlywheelRPM(testFlywheelRpm);
                    if (bPressed) Flywheel.INSTANCE.stop();
                    break;

                case TURRET:
                    // Temporary wide limits during bench calibration
                    Turret.MIN_ANGLE_DEG = -180.0;
                    Turret.MAX_ANGLE_DEG = 180.0;
                    if (upPressed) {
                        testTurretTargetDeg += 5.0;
                        Turret.INSTANCE.setTargetAngle(testTurretTargetDeg);
                    }
                    if (downPressed) {
                        testTurretTargetDeg -= 5.0;
                        Turret.INSTANCE.setTargetAngle(testTurretTargetDeg);
                    }
                    if (rightPressed) {
                        testTurretTargetDeg += 1.0;
                        Turret.INSTANCE.setTargetAngle(testTurretTargetDeg);
                    }
                    if (leftPressed) {
                        testTurretTargetDeg -= 1.0;
                        Turret.INSTANCE.setTargetAngle(testTurretTargetDeg);
                    }
                    if (Math.abs(gamepad1.left_stick_x) > 0.08) {
                        Turret.INSTANCE.setManualPower(gamepad1.left_stick_x * 0.25);
                    } else if (!Turret.INSTANCE.isPositionMode()) {
                        Turret.INSTANCE.setManualPower(0.0);
                    }
                    if (bPressed) {
                        testTurretTargetDeg = 0.0;
                        Turret.INSTANCE.setTargetAngle(0.0);
                    }
                    if (yPressed) {
                        Turret.INSTANCE.resetZero();
                        testTurretTargetDeg = 0.0;
                    }
                    break;

                case HOOD:
                    if (upPressed) Hood.INSTANCE.adjustPosition(0.02);
                    if (downPressed) Hood.INSTANCE.adjustPosition(-0.02);
                    if (rightPressed) Hood.INSTANCE.adjustPosition(0.005);
                    if (leftPressed) Hood.INSTANCE.adjustPosition(-0.005);
                    if (aPressed) Hood.INSTANCE.setPosition(0.50);
                    break;

                case BLOCKER:
                    if (upPressed) Blocker.INSTANCE.adjustPosition(0.02);
                    if (downPressed) Blocker.INSTANCE.adjustPosition(-0.02);
                    if (rightPressed) Blocker.INSTANCE.adjustPosition(0.005);
                    if (leftPressed) Blocker.INSTANCE.adjustPosition(-0.005);
                    if (aPressed) Blocker.INSTANCE.open();
                    if (bPressed) Blocker.INSTANCE.close();
                    if (yPressed) {
                        // Save current position as OPEN (with Left Stick pressed) or CLOSED
                        if (gamepad1.left_stick_button) {
                            Blocker.OPEN_POSITION = Blocker.INSTANCE.getPosition();
                        } else {
                            Blocker.CLOSED_POSITION = Blocker.INSTANCE.getPosition();
                        }
                    }
                    break;

                case INTAKE_MOTOR:
                    if (upPressed) testIntakePower = Math.min(1.0, testIntakePower + 0.1);
                    if (downPressed) testIntakePower = Math.max(-1.0, testIntakePower - 0.1);
                    if (aPressed) Transfer.INSTANCE.setIntakePower(testIntakePower);
                    if (bPressed) Transfer.INSTANCE.stopIntake();
                    if (yPressed) Transfer.INSTANCE.setIntakePower(-testIntakePower);
                    break;

                case FEED_MOTOR:
                    if (upPressed) testFeedPower = Math.min(1.0, testFeedPower + 0.1);
                    if (downPressed) testFeedPower = Math.max(-1.0, testFeedPower - 0.1);
                    if (aPressed) Transfer.INSTANCE.setFeedPower(testFeedPower);
                    if (bPressed) Transfer.INSTANCE.stopFeed();
                    if (yPressed) Transfer.INSTANCE.setFeedPower(-testFeedPower);
                    break;

                case INTAKE_RAMP:
                    if (upPressed) IntakeRamp.INSTANCE.adjustPosition(0.02);
                    if (downPressed) IntakeRamp.INSTANCE.adjustPosition(-0.02);
                    if (rightPressed) IntakeRamp.INSTANCE.adjustPosition(0.005);
                    if (leftPressed) IntakeRamp.INSTANCE.adjustPosition(-0.005);
                    if (aPressed) IntakeRamp.INSTANCE.lower();
                    if (bPressed) IntakeRamp.INSTANCE.raise();
                    break;
            }

            // Record snapshot to RobotLog when X is pressed
            if (xPressed) {
                lastSavedBanner = String.format(Locale.US,
                        "Hood=%.3f | Blocker=%.3f | Ramp=%.3f | Turret=%.1f deg",
                        Hood.INSTANCE.getPosition(),
                        Blocker.INSTANCE.getPosition(),
                        IntakeRamp.INSTANCE.getPosition(),
                        Turret.INSTANCE.getAngleDegrees());

                RobotLog.ii(LOG_TAG, "=== SHOOTER & INTAKE CALIBRATION SNAPSHOT ===");
                RobotLog.ii(LOG_TAG, "Flywheel Target=%.0f RPM | Actual=%.0f RPM",
                        Flywheel.INSTANCE.getTargetFlywheelRPM(), Flywheel.INSTANCE.getFlywheelRPM());
                RobotLog.ii(LOG_TAG, "Turret Angle=%.2f deg | Ticks=%d",
                        Turret.INSTANCE.getAngleDegrees(), Turret.INSTANCE.getEncoderTicks());
                RobotLog.ii(LOG_TAG, "Hood Position=%.3f", Hood.INSTANCE.getPosition());
                RobotLog.ii(LOG_TAG, "Blocker Position=%.3f (CLOSED=%.3f, OPEN=%.3f)",
                        Blocker.INSTANCE.getPosition(), Blocker.CLOSED_POSITION, Blocker.OPEN_POSITION);
                RobotLog.ii(LOG_TAG, "IntakeRamp Position=%.3f (RAISED=%.3f, LOWERED=%.3f)",
                        IntakeRamp.INSTANCE.getPosition(), IntakeRamp.RAISED_POSITION, IntakeRamp.LOWERED_POSITION);
            }

            // Update subsystem periodic loops
            Flywheel.INSTANCE.periodic();
            Turret.INSTANCE.periodic();
            Hood.INSTANCE.periodic();
            Blocker.INSTANCE.periodic();
            Transfer.INSTANCE.periodic();
            IntakeRamp.INSTANCE.periodic();

            // Telemetry readout
            telemetry.addLine("==========================================");
            telemetry.addData("ACTIVE MODE [LB/RB]", currentMode.label);
            telemetry.addData("Last Saved [Press X]", lastSavedBanner);
            telemetry.addLine("==========================================");

            telemetry.addData("1. Flywheel RPM", "Target=%.0f | Actual=%.0f | Ready=%b [A=Run, B=Stop]",
                    testFlywheelRpm, Flywheel.INSTANCE.getFlywheelRPM(), Flywheel.INSTANCE.isReadyToShoot());
            telemetry.addData("2. Turret", "Angle=%.2f deg (Target=%.1f) | Ticks=%d [Stick/D-Pad, Y=Zero]",
                    Turret.INSTANCE.getAngleDegrees(), testTurretTargetDeg, Turret.INSTANCE.getEncoderTicks());
            telemetry.addData("3. Hood Servo", "Pos=%.3f [D-Pad U/D +/-0.02, L/R +/-0.005]",
                    Hood.INSTANCE.getPosition());
            telemetry.addData("4. Blocker Servo", "Pos=%.3f | Open=%b [A=Open, B=Close, D-Pad=Jog]",
                    Blocker.INSTANCE.getPosition(), Blocker.INSTANCE.isOpen());
            telemetry.addData("5. Intake (1620)", "Pwr=%.2f | RPM=%.0f [A=In, Y=Rev, B=Stop]",
                    Transfer.INSTANCE.getIntakePower(), Transfer.INSTANCE.getIntakeRPM());
            telemetry.addData("6. Feed (1620)", "Pwr=%.2f | RPM=%.0f [A=Feed, Y=Rev, B=Stop]",
                    Transfer.INSTANCE.getFeedPower(), Transfer.INSTANCE.getFeedRPM());
            telemetry.addData("7. Intake Ramp", "Pos=%.3f | Lowered=%b [A=Lower, B=Raise, D-Pad=Jog]",
                    IntakeRamp.INSTANCE.getPosition(), IntakeRamp.INSTANCE.isLowered());

            telemetry.update();

            prevLb = gamepad1.left_bumper;
            prevRb = gamepad1.right_bumper;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            sleep(20);
        }

        Flywheel.INSTANCE.stop();
        Turret.INSTANCE.stop();
        Transfer.INSTANCE.stop();
    }
}
