package org.firstinspires.ftc.teamcode.subsystems;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * 
 */
public class Drivetrain implements Subsystem {


    public static final Drivetrain INSTANCE = new Drivetrain();

    // MOTOR CONFIGURATION NAMES
    // These strings are the names that we will give the motors in the FTC Robot Controller configuration.
    // IMPORTANT: The names here MUST exactly match the names configured on the robot.
    private final String rightFrontName = "rightFront";
    private final String leftFrontName = "leftFront";
    private final String rightBackName = "rightBack";
    private final String leftBackName = "leftBack";

    //MOTOR OBJECTS
    //These variables represent the actual motors.
    //Left side is reversed.
    private final MotorEx rightFrontMotor = new MotorEx(rightFrontName);

    private final MotorEx leftFrontMotor = new MotorEx(leftFrontName).reversed();

    private final MotorEx rightBackMotor = new MotorEx(rightBackName);

    private final MotorEx leftBackMotor = new MotorEx(leftBackName).reversed();


    private Drivetrain() {
        // The motors are already created above.
    }

    /**
     * 
     * @return 
     */
    public MotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }

    /**
     * 
     * @return 
     */
    public MotorEx getRightFrontMotor() {
        return rightFrontMotor;
    }

    /**
     * 
     * @return 
     */
    public MotorEx getLeftBackMotor() {
        return leftBackMotor;
    }

    /**
     * 
     * @return 
     */
    public MotorEx getRightBackMotor() {
        return rightBackMotor;
    }

    /**
     * Drives the mecanum drivetrain using robot-centric forward, strafe (right+), and turn (clockwise+) inputs.
     */
    public void drive(double forward, double strafe, double turn) {
        double leftFrontPower = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftBackPower = forward - strafe + turn;
        double rightBackPower = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        ));

        leftFrontMotor.setPower(leftFrontPower / max);
        rightFrontMotor.setPower(rightFrontPower / max);
        leftBackMotor.setPower(leftBackPower / max);
        rightBackMotor.setPower(rightBackPower / max);
    }

    /** Stops all four drive motors immediately. */
    public void stop() {
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
    }

}
