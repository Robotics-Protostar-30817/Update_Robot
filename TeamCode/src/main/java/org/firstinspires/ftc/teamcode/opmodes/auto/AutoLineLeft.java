package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import static com.pedropathing.api.Paths.line;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Pedro Pathing 3 forward/backward and odometry diagnostic.
 *
 * Before START: place robot at the physical origin, heading 0.
 * A: follow a straight path from the current estimated pose to (12, 0, 0).
 * B: follow a straight path from the current estimated pose to (0, 0, 0).
 * The robot must have clear space to move; paths are in inches.
 */
@Autonomous(name = "Pedro Strafe Left/Right Drive Test", group = "Test")
public class AutoLineLeft extends OpMode {

    private static final Pose START_POSE = new Pose(0, 0, 0);
    private static final Pose FORWARD_POSE = new Pose(0, 24, 0);

    private Follower follower;
    private boolean lastA;
    private boolean lastB;
    private String command = "Idle";
    private boolean pathStarted = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        telemetry.addLine("Pedro Pathing 3 left/right test");
        telemetry.addLine("Position robot at (0,0), facing heading 0 before START.");
        telemetry.addLine("A: go to (0,24); B: return to (0,0)");
        telemetry.addLine("Check that pushing forward increases X, left increases Y.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        command = "Idle";
        pathStarted = false;
    }

    /** Follow a new line from the *measured current* pose to the target. */
    private void goTo(Pose target, String description) {
        Pose from = follower.pose();
        // Pedro Pathing 3: a line with constant target heading.
        Path path = line(from, target).constant(target.heading());
        follower.follow(path);
        command = description;
        pathStarted = true;
    }

    @Override
    public void loop() {
        follower.update();

        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;

        // Do not replace an active path with a new command.
        // A and B can be used again after the current path ends.
        if (!pathStarted || follower.atParametricEnd()) {
            if (aPressed) {
                goTo(FORWARD_POSE, "Strafe left to (0, 24)");
            } else if (bPressed) {
                goTo(START_POSE, "Return to (0, 0)");
            }
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;

        Pose pose = follower.pose();
        telemetry.addData("Command", command);
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("At path end", follower.atParametricEnd());
        telemetry.addLine("A: drive to Y=24 | B: drive to origin");
        telemetry.update();
    }
}
