package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem.TagGroup;

// TODO IMPORTANT: upload the paths from path planner
//   should be named: straight, side, side2, side3
public record Auton(boolean flipAlliance, boolean flipSide) {

    /**
     * Move straight out of the starting zone and do nothing.
     */
    Command c_straight() {
        return Component.chassis.getAutonomousCommand("straight", true, flipAlliance, false);
    }

    /**
     * Move straight out of the starting zone, align with the center of the reef, and outtake.
     */
    Command c_straightCoral() {
        return new SequentialCommandGroup(
            Component.chassis.getAutonomousCommand("straight", true, flipAlliance, false),
            Component.vision.c_align(TagGroup.REEF_INNER_CENTER, 1),
            Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L1),
            Component.elevator.c_outtake()
        );
    }

    /**
     * Move diagonally from the starting zone, align with the inner diagonal of the reef, and outtake.
     */
    Command c_sideCoral() {
        return new SequentialCommandGroup(
            Component.chassis.getAutonomousCommand("side", true, flipAlliance, flipSide),
            Component.vision.c_align(TagGroup.REEF_INNER_DIAGONAL, 1),
            Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L1),
            Component.elevator.c_outtake()
        );
    }

    /**
     * 1. Move diagonally from the starting zone 2. Align with the inner diagonal of the reef 3. Outtake 4. Move to and
     * align with the coral intake station 5. Wait 2 seconds for intake 6. Move to and align with the outer diagonal of
     * the reef 7. Outtake
     */
    Command c_sideCoralFancy() {
        return new SequentialCommandGroup(
            c_sideCoral(),
            Component.chassis.getAutonomousCommand("side2", true, flipAlliance, flipSide),
            new WaitCommand(2),
            Component.elevator.c_intake(),
            Component.chassis.getAutonomousCommand("side3", true, flipAlliance, flipSide),
            Component.vision.c_align(TagGroup.REEF_OUTER_DIAGONAL, 1),
            Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L1),
            Component.elevator.c_outtake()
        );
    }
}
