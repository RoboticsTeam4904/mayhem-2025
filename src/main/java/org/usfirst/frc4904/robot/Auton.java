package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem.TagGroup;

public static class Auton {

    /**
     * Move straight out of the starting zone and do nothing.
     */
    static Command c_straight() {
        return Component.chassis.getAutonomousCommand("straight", true, Robot.AutonConfig.getFlipAlliance(), false);
    }

    /**
     * Move straight out of the starting zone, align with the center of the reef, and outtake.
     */
    static Command c_straightCoral() {
        return new SequentialCommandGroup(
            Component.chassis.getAutonomousCommand("straight", true, Robot.AutonConfig.getFlipAlliance(), false),
            Component.vision.c_align(TagGroup.REEF_INNER_CENTER, -1),
            Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L1)
        );
    }

    /**
     * Move diagonally from the starting zone, align with the inner diagonal of the reef, and outtake.
     */
    static Command c_sideCoral() {
        return new SequentialCommandGroup(
            Component.chassis.getAutonomousCommand("side", true, Robot.AutonConfig.getFlipAlliance(), Robot.AutonConfig.getFlipSide()),
            Component.vision.c_align(TagGroup.REEF_INNER_DIAGONAL, -1),
            Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L1)
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
    static Command c_sideCoralFancy() {
        return new SequentialCommandGroup(
            c_sideCoral(),
            Component.chassis.getAutonomousCommand("side2", true, Robot.AutonConfig.getFlipAlliance(), Robot.AutonConfig.getFlipSide()),
            new WaitCommand(2),
            Component.elevator.c_intake(),
            Component.chassis.getAutonomousCommand("side3", true, Robot.AutonConfig.getFlipAlliance(), Robot.AutonConfig.getFlipSide()),
            Component.vision.c_align(TagGroup.REEF_OUTER_DIAGONAL, -1),
            Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L1)
        );
    }
}
