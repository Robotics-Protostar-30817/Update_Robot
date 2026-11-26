package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

//ivy scheduler of Pedro Pathing v3 is adopted.
/**
 * 
 */
@Autonomous(name = "Pedro Pathing ivy testing", group = "Test")
public class LargeExampleAuto extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(24,24,0);
    private final Pose scoreApproachPose = poseFactory.of(40, 24, 0);
    private final Pose scorePose = poseFactory.of(40,24,90);
    private final Pose parkPose = poseFactory.of(20,100,90);

    private Path startToScore(){
        return line(startPose, scoreApproachPose).constant(0);
    }
    private Path park(){
        return line(scorePose, parkPose).constant(Math.toRadians(90));
    }

    private Command autoRoutine(){
        return sequential(follow(follower,startToScore()),
                //add other mechanism commands
                instant(() -> follower.hold(scorePose)),//turn from 0 to 90
                waitMs(1500),
                follow(follower,park()));
    }
    @Override
    /**
     * 
     */
    public void init(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    /**
     * 
     */
    public void start(){
        schedule(autoRoutine());
    }

    @Override
    /**
     * 
     */
    public void loop(){
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y",follower.pose().y());
        telemetry.addData("Headhing",Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode",follower.mode());
        telemetry.update();

    }
}
