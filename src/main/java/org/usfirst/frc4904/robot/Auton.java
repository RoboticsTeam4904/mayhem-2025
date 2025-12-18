package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc4904.robot.RobotMap.Component;

public class Auton {

    public static Command c_jankStraight() {
        return new SequentialCommandGroup(
            new WaitCommand(8),
            Component.chassis.c_driveRobotRelative(-1.5, 0, 0).withTimeout(2), // kitter is sponsored by 0.749
            Component.chassis.c_stop()
        );
    }

    public static Command c_jankReverse() {
        return new SequentialCommandGroup(
            new WaitCommand(8),
            Component.chassis.c_driveRobotRelative(1.5, 0, 0).withTimeout(2), // kitter does not approve of 7.5
            Component.chassis.c_stop()
        );
    }
}
