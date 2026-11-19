# Robotics Protostar — FTC Team 30817

## Robot Software Project

This repository contains the robot control software developed by **Robotics Protostar, FTC Team 30817**.

The project is built on the FIRST Tech Challenge (FTC) SDK and uses Java, NextFTC, and Pedro Pathing for robot control and autonomous navigation.

Our robot software includes:

- **Drivetrain and Navigation:** Mecanum drive, goBILDA Pinpoint odometry, and Pedro Pathing.
- **Vision:** USB camera and AprilTag-based target detection.
- **POLLEN Collection:** Intake ramp and transfer motors for collecting and transporting POLLEN.
- **Shooting System:** Turret, adjustable hood, flywheel, and blocker gate.
- **Automatic Aiming:** Vision-guided aiming and calibrated shooting parameters.
- **Testing and Calibration:** Individual subsystem tests and integrated robot testing.

## Documentation

| Resource | Description |
|---|---|
| [Team GitBook](https://robotics-protostar-30817.gitbook.io/ftc-programming-manual/) | Robot design, programming guides, subsystem documentation, and testing procedures |
| [Team Javadoc](https://robotics-protostar-30817.github.io/Update_Robot/javadoc/) | Java API reference for the team's robot software |
| [Robot Source Code](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) | Java source files for the robot |

## Project Structure

```text
Update_Robot/
├── TeamCode/            # Team-developed robot software
├── FtcRobotController/  # FTC Robot Controller application
├── docs/                # Project documentation
├── doc/                 # Additional documentation
├── README.md            # Project overview
└── build.gradle         # Gradle build configuration
```

## Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/Robotics-Protostar-30817/Update_Robot.git
   ```
2. Open the project in Android Studio.
3. Allow Gradle to synchronize and resolve dependencies.
4. Configure the robot hardware in the FTC Robot Controller.
5. Review the team GitBook for subsystem setup and calibration procedures.
6. Test individual subsystems before running integrated robot programs.

## Robot Testing and Calibration

After hardware installation or modification, test the robot in the following order:

1. Turret motor and encoder calibration
2. Hood servo and launch-angle calibration
3. Blocker and intake ramp servo calibration
4. Flywheel and transfer motor testing
5. Camera and AprilTag calibration
6. Pinpoint odometry and Pedro Pathing tuning
7. Shot calibration and automatic aiming
8. Complete shooter integration testing

**Safety:** Perform initial tests without POLLEN. Verify motor directions, servo limits, and emergency stopping behavior before attempting shooting tests.

## Development Workflow

Team members should synchronize their local repository before making changes.

```bash
git pull --ff-only
```

After testing and reviewing changes:

```bash
git add .
git commit -m "Describe changes"
git push
```

Use descriptive commit messages. Review changes before committing, and coordinate modifications to shared robot subsystems.

## FTC SDK and Official Resources

This project is based on the official [FIRST Tech Challenge Robot Controller SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

Official resources:

- [FTC Documentation](https://ftc-docs.firstinspires.org/)
- [FTC SDK Javadoc](https://javadoc.io/doc/org.firstinspires.ftc)
- [FTC SDK Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases)

The FTC SDK and third-party libraries remain subject to their respective licenses. See the repository's [LICENSE](LICENSE) for applicable license information.

---

**Robotics Protostar — FTC Team 30817**