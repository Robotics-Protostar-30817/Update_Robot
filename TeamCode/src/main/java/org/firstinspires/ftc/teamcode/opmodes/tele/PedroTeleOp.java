package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Minimal robot-centric Pedro Pathing TeleOp drivetrain test.
 *
 * Gamepad 1:
 *   Left stick Y: forward/backward
 *   Left stick X: strafe
 *   Right stick X: turn
 *
 * Uses the existing Pedro Mecanum + Pinpoint configuration in Constants.java.
 * No NextFTC scheduler, other drivetrain subsystem, or shooter is initialized.
 */
@TeleOp(name = "Pedro TeleOp - Drive Test", group = "Test")
public class PedroTeleOp extends OpMode {

    private static final double DEADZONE = 0.05;
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        telemetry.addLine("Pedro robot-centric drive test ready");
        telemetry.addLine("Left stick: forward/back and strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.addLine("Lift wheels off floor for first direction test.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = deadzone(-gamepad1.left_stick_y);
        double strafe = deadzone(gamepad1.left_stick_x);
        double turn = deadzone(gamepad1.right_stick_x);

        // Update Pedro localization before commanding manual motor powers.
        // The sign mapping below matches the PEDRO branch in BaseTeleOp.java.
        // Verify directions on your robot before driving at full power.
        follower.update();
        follower.drivetrain.drive(new DrivePowers(forward, -strafe, -turn), false);

        Pose pose = follower.pose();
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe (right +)", "%.2f", strafe);
        telemetry.addData("Turn (right +)", "%.2f", turn);
        if (pose != null) {
            telemetry.addData("Pedro X (in)", "%.2f", pose.x());
            telemetry.addData("Pedro Y (in)", "%.2f", pose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        }
        telemetry.update();
    }

    private static double deadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
            follower.stop();
        }
    }
}
