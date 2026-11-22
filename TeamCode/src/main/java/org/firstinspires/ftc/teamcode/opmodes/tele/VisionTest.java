package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.VisionPortal;

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

        VisionPortal p = Robot.INSTANCE.vision.getVisionPortal();
        if (p!=null){
            telemetry.addData("Camera State/FPS", "%s | %.1f FPS",
                    p.getCameraState(),p.getFps());
            WhiteBalanceControl wb = p.getCameraControl(WhiteBalanceControl.class);
            if (wb !=null)
            {
                telemetry.addData("White Balance", "%s (%dK)",
                        wb.getMode(),wb.getWhiteBalanceTemperature());
            }
        }
        //Test Red cell only
        //Vision.Cell best = Robot.INSTANCE.vision.getBestScorableCell();
        Vision.Cell best = Robot.INSTANCE.vision.getBestCell(Vision.Cell.Color.RED);
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

        boolean redScoringScorable = Robot.INSTANCE.vision.getBestCell(
                Vision.Cell.Color.RED, Vision.Cell.Location.SCORING) != null;
        boolean blueScoringScorable = Robot.INSTANCE.vision.getBestCell(
                Vision.Cell.Color.BLUE, Vision.Cell.Location.SCORING) != null;

        telemetry.addLine("------------------------");
        telemetry.addData("Total Scorable Visible", Robot.INSTANCE.vision.getScorableCells().size());
        telemetry.addData("Red Audience", Robot.INSTANCE.vision.isRedAudienceScorable() ? "SCORABLE" : "--");
        telemetry.addData("Red Scoring", redScoringScorable ? "SCORABLE" : "--");
        telemetry.addData("Blue Audience", Robot.INSTANCE.vision.isBlueAudienceScorable() ? "SCORABLE" : "--");
        telemetry.addData("Blue Scoring", blueScoringScorable ? "SCORABLE" : "--");
        telemetry.update();
    }

    @Override
    public void stop() {
        Robot.INSTANCE.vision.close();
    }
}