package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.robot.RobotMap.Component;

public class Auton {

    public static boolean getFlipSide() {
        return Robot.AutonConfig.FLIP_SIDE;
    }

    /**
     * Move straight out of the starting zone and do nothing.
     */
    public static Command c_straight() {
        return Component.chassis.getAutonomousCommand("straight", true, false);
    }

}
