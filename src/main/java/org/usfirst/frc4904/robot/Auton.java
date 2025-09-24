package org.usfirst.frc4904.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc4904.robot.RobotMap.Component;

public class Auton {

    public static boolean getFlipSide() {
        return Robot.AutonConfig.FLIP_SIDE;
    }

    /**
     * Move straight out of the starting zone and do nothing.
     */
    public static Command c_straight() {
        ChassisSpeeds speed = new ChassisSpeeds(-0.5, 0, 0);

        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new RunCommand(() -> Component.chassis.drive(speed))
            ),
            new InstantCommand(Component.chassis::stop)
        );
    }

}
