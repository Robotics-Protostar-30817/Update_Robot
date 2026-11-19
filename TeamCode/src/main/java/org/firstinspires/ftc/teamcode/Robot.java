package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.CompleteShooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRamp;
import org.firstinspires.ftc.teamcode.subsystems.Odom;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Main robot container class implementing the singleton pattern.
 * Provides centralized access to all robot subsystems.
 */
public class Robot {

    public static final Robot INSTANCE = new Robot();

    // TeleOp Drivetrain & Odometry (do NOT init in Pedro Pathing Auto; Pedro owns those devices)
    public final Drivetrain dt;
    public final Odom odom;

    // Vision & Shooter Subsystems
    public final Vision vision;
    public final Flywheel flywheel;
    public final Turret turret;
    public final Hood hood;
    public final Transfer transfer;
    public final IntakeRamp intakeRamp;
    public final Blocker blocker;
    public final AutoAim autoAim;
    public final CompleteShooter completeShooter;

    private Robot() {
        dt = Drivetrain.INSTANCE;
        odom = Odom.INSTANCE;
        vision = Vision.INSTANCE;
        flywheel = Flywheel.INSTANCE;
        turret = Turret.INSTANCE;
        hood = Hood.INSTANCE;
        transfer = Transfer.INSTANCE;
        intakeRamp = IntakeRamp.INSTANCE;
        blocker = Blocker.INSTANCE;
        autoAim = AutoAim.INSTANCE;
        completeShooter = CompleteShooter.INSTANCE;
    }

    /**
     * Initializes Vision and all 7 shooter/intake actuators in one place.
     * Safe to call in both Autonomous (alongside Pedro Pathing) and TeleOp.
     */
    public void initShooterAndVision(HardwareMap hardwareMap,
                                     Telemetry telemetry,
                                     Vision.Cell.Color alliance,
                                     Vision.Cell.Location targetLocation,
                                     boolean useNextFtcScheduler) {
        turret.init(hardwareMap.get(DcMotorEx.class, "turret"));
        hood.init(hardwareMap.get(Servo.class, "hood"));
        flywheel.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        transfer.init(
                hardwareMap.get(DcMotorEx.class, "transfer1"),
                hardwareMap.get(DcMotorEx.class, "transfer2")
        );
        blocker.init(hardwareMap.get(Servo.class, "blocker"));
        intakeRamp.init(hardwareMap.get(Servo.class, "intake"));

        vision.init(hardwareMap, telemetry);
        vision.setEnableTelemetry(false);

        completeShooter.init(alliance, targetLocation);
        completeShooter.setCallSubPeriodic(!useNextFtcScheduler);
    }

    /** Returns all Vision and Shooter subsystems for NextFTC SubsystemComponent registration. */
    public Subsystem[] shooterAndVisionSubsystems() {
        return new Subsystem[]{
                vision,
                turret,
                hood,
                flywheel,
                transfer,
                blocker,
                intakeRamp,
                autoAim,
                completeShooter
        };
    }
}