package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
//import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "NextFTC scheduling testing", group = "Test")
public class LargeExampleNextFTCAuto extends NextFTCOpMode{


    private Follower follower;
    //define paths
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(24,24,0);
    private final Pose scoreApproachPose = poseFactory.of(40, 24, 0);
    private final Pose scorePose = poseFactory.of(40,24,90);
    private final Pose parkPose = poseFactory.of(15  ,100,90);

    private Path startToScore(){
        return line(startPose, scoreApproachPose).constant(0);
    }
    private Path park(){
        return line(scorePose, parkPose).constant(Math.toRadians(90));
    }

    //The command lifecycle is:
    //- setStart() executes once and starts the path.
    //- setIsDone() checks whether the follower has finished.
    //- setStop() stops the follower if the command is interrupted.
    private Command followPathCommand(Path path) {
        ElapsedTime timer = new ElapsedTime();

        return new LambdaCommand()
                .setStart(() -> {
                    timer.reset();
                    follower.follow(path);
                })
                .setIsDone(() ->
                        timer.seconds() >= 0.05
                                && !follower.isBusy()
                )
                .setStop(interrupted -> {
                    if (interrupted) {
                        follower.stop();
                    }
                });
    }

    //This hold function introduces two conditions:
    //- The robot is within 3° of the target heading.
    //- Or 3 seconds have elapsed.
    //The timeout prevents the command from waiting indefinitely.
    // However, reaching the timeout should be treated as a failed turn in
    // a production autonomous, rather than silently proceeding.
    // Also, this command checks heading only;
    // the robot's X/Y position and whether it has settled should be
    // checked for a more robust implementation.
    private Command turnToPoseCommand(Pose p) {
        ElapsedTime timer = new ElapsedTime();

        return new LambdaCommand()
                .setStart(() -> {
                    timer.reset();
                    follower.hold(p);
                })
                .setIsDone(() -> {
                    double error = Math.atan2(
                            Math.sin(p.heading() - follower.pose().heading()),
                            Math.cos(p.heading() - follower.pose().heading())
                    );

                    return Math.abs(Math.toDegrees(error)) < 3.0
                            || timer.seconds() >= 3.0;
                })
                .setStop(interrupted -> {
                    if (interrupted) {
                        follower.stop();
                    }
                });
    }

    private Command autoRoutine() {
        return new SequentialGroup(
                followPathCommand(startToScore()),
                turnToPoseCommand(scorePose),
                new Delay(1.5),
               followPathCommand(park())
        );
    }

    @Override
    public void onInit() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
