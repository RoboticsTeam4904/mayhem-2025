package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc4904.robot.RobotMap.Component;

public class Auton {

    // actually moves backwards - robot must be placed physically backwards on the field
    public static Command c_jankStraight() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(-0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    // actually moves forwards
    public static Command c_jankReverse() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }
}
