package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "VisionTest", group = "Test")
public class VisionTest extends OpMode {

    @Override
    public void init() {
        Robot.INSTANCE.vision.init(hardwareMap, telemetry);
        Robot.INSTANCE.vision.setEnableTelemetry(false);
        telemetry.addLine("Vision initialized");
        telemetry.addLine("Press Start and point Webcam at any Cell");
        telemetry.update();
    }

    @Override
    public void loop() {
        Robot.INSTANCE.vision.periodic();

        Vision.Cell best = Robot.INSTANCE.vision.getBestScorableCell();
        if (best != null) {
            telemetry.addLine("== BEST SCORABLE CELL ==");
            telemetry.addData("Name & Color", best.getNameAndColor());
            telemetry.addData("Name", best.getName());
            telemetry.addData("Color", best.getColor());
            telemetry.addData("Location", best.getLocation());
            telemetry.addData("Scorable", best.isScorable() ? "YES" : "NO");
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    best.getX(), best.getY(), best.getZ()));
        } else {
            telemetry.addLine("No Scorable Cell detected");
        }

        telemetry.addLine("------------------------");
        telemetry.addData("Total Scorable Visible", Robot.INSTANCE.vision.getScorableCells().size());
        telemetry.addData("Red Audience", Robot.INSTANCE.vision.isRedAudienceScorable() ? "SCORABLE" : "--");
        telemetry.addData("Red Scoring", Robot.INSTANCE.vision.isRedScoringScorable() ? "SCORABLE" : "--");
        telemetry.addData("Blue Audience", Robot.INSTANCE.vision.isBlueAudienceScorable() ? "SCORABLE" : "--");
        telemetry.addData("Blue Scoring", Robot.INSTANCE.vision.isBlueScoringScorable() ? "SCORABLE" : "--");
        telemetry.update();
    }

    @Override
    public void stop() {
        Robot.INSTANCE.vision.close();
    }
}