package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem.TagGroup;

public class Auton {

    public static boolean getFlipSide() {
        return Robot.AutonConfig.FLIP_SIDE;
    }

    // public static Command c_jank() {
    //     return new ParallelDeadlineGroup(
    //         new WaitCommand(1),
    //         new InstantCommand(() -> RobotMap.Component.chassis.drive(
    //             ChassisSpeeds.fromRobotRelativeSpeeds(
    //                 1.0,
    //                 0.0,
    //                 0.0,
    //                 Rotation2d.kZero
    //             )
    //         )) 
    //     );
    // }

    /**
     * Move straight out of the starting zone and do nothing.
     */
    public static Command c_straight() {
        return Component.chassis.getAutonomousCommand("straight", true, false);
    }

     /**
     * Move straight out of the starting zone, align with the center of the reef, and outtake.
     */
    public static Command c_straightCoral() {
        // TODO super janky
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new Command() {
                    @Override
                    public void execute() {
                        Component.chassis.drive(new ChassisSpeeds(0, 3, 0));
                    }
                }
            ),
            new InstantCommand(() -> Component.chassis.drive(new ChassisSpeeds(0, 0, 0))),
            // Component.chassis.getAutonomousCommand("straight", true, getFlipAlliance(), false),
            Component.vision.c_align(TagGroup.REEF_INNER_DIAGONAL, -1)
            //Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2)
        );
    }

    // /**
    //  * Move straight out of the starting zone, align with the center of the reef, and outtake.
    //  */
    // public static Command c_straightCoral() {
    //     return new SequentialCommandGroup(
    //         // Component.chassis.getAutonomousCommand("straight", true, getFlipAlliance(), false),
    //         Component.vision.c_align(TagGroup.REEF_INNER_CENTER, -1),
    //         Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2)
    //     );
    // }

    /**
     * Move diagonally from the starting zone, align with the inner diagonal of the reef, and outtake.
     */
    public static Command c_sideCoral() {
        return new SequentialCommandGroup(
            Component.chassis.getAutonomousCommand("side", true, getFlipSide()),
            Component.vision.c_align(TagGroup.REEF_INNER_DIAGONAL, -1)
            //Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2)
        );
    }

    /**
     * <ol>
     *   <li> Move diagonally from the starting zone
     *   <li> Align with the inner diagonal of the reef
     *   <li> Outtake
     *   <li> Move to and align with the coral intake station
     *   <li> Wait 2 seconds for intake
     *   <li> Move to and align with the outer diagonal of the reef
     *   <li> Outtake
     * </ol>
     */
    public static Command c_sideCoralFancy() {
        return new SequentialCommandGroup(
            c_sideCoral(),
            Component.chassis.getAutonomousCommand("side2", true, getFlipSide()),
            new WaitCommand(2),
            //Component.elevator.c_intake(),
            Component.chassis.getAutonomousCommand("side3", true, getFlipSide()),
            Component.vision.c_align(TagGroup.REEF_OUTER_DIAGONAL, -1)
            //Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2)
        );
    }
}
