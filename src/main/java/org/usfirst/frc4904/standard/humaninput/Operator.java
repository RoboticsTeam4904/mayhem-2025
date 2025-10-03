package org.usfirst.frc4904.standard.humaninput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;

/**
 * Operator specific version of HumanInterface
 *
 */
public abstract class Operator extends HumanInput {

    /**
     * Slowly move the elevator downwards, bypassing the software stop.
     * Then, zero the elevator encoder to the current position when the command ends.
     *
     * @return the command
     */
    static protected Command c_manualElevatorZero() {
        return Component.elevator.runEnd(
            // repeatedly:
            () -> Component.elevator.setVoltage(-2, true),
            // on end:
            () -> {
                Component.elevator.setVoltage(0);
                Component.elevatorEncoder.reset();
            }
        );
    }

    static protected Command c_resetOdometry() {
        return new InstantCommand(() -> Component.chassis.resetOdometry(Pose2d.kZero));
    }

    public Operator(String name) {
        super(name);
    }
}
