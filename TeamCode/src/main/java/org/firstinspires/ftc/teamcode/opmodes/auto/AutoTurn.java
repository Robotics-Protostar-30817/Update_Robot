package org.firstinspires.ftc.teamcode.opmodes.auto;
//package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Pedro Turn Test", group = "Test")
public class AutoTurn extends OpMode {
    private Follower follower;
    private static final Pose START_POSE = Pose.zero();

    private boolean lastA;
    private boolean lastB;
    private boolean lastX;
    private double targetHeading = 0.0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        telemetry.addLine("Pedro Pathing 3 - Turn Test");
        telemetry.addLine("A: face +90 deg (left)");
        telemetry.addLine("B: face -90 deg (right)");
        telemetry.addLine("X: face 0 deg");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
        lastA = lastB = lastX = false;
        targetHeading = 0.0;
    }

    private void turnTo(double headingRadians) {
        Pose current = follower.pose();
        targetHeading = headingRadians;
        // Pedro Pathing 3: hold a pose at the current X/Y with a new heading.
        follower.hold(new Pose(current.x(), current.y(), targetHeading));
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        boolean xPressed = gamepad1.x && !lastX;

        if (aPressed) {
            turnTo(Math.toRadians(90.0));
        } else if (bPressed) {
            turnTo(Math.toRadians(-90.0));
        } else if (xPressed) {
            turnTo(0.0);
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;

        follower.update();
        Pose pose = follower.pose();
        telemetry.addLine("Pedro Pathing 3 - Turn Test");
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Target heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Heading error (deg)", Math.toDegrees(
                Math.atan2(Math.sin(targetHeading - pose.heading()),
                        Math.cos(targetHeading - pose.heading()))));
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        // The FTC runtime stops motor outputs when the OpMode ends.
        // Do not issue new follower commands here.
    }
}
