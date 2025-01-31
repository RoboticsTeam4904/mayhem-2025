package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

public class ElevatorSubsystem extends MultiMotorSubsystem {

    // TODO maybe remove once feedforward is added
    public static final double UP_VOLTAGE = 1;
    public static final double DOWN_VOLTAGE = 1;

    // make sure that all values defined in this enum are added to the 'positions' map in the constructor
    public enum Position {
        INTAKE,
        OUTTAKE,
    }

    public static HashMap<Position, Double> positions = new HashMap<>();

    public ElevatorSubsystem(CANTalonFX motor1, CANTalonFX motor2) {
        super(
            new CANTalonFX[] { motor1, motor2 },
            new double[] { 1, -1 },
            UP_VOLTAGE,
            DOWN_VOLTAGE
        );
        // TODO change (obviously)
        positions.put(Position.INTAKE, 0.0);
        positions.put(Position.OUTTAKE, 4.0);
    }

    public Command c_gotoHeight(double height) {
        return new Noop(); // TODO
    }

    public Command c_gotoPosition(Position pos) {
        Double height = positions.get(pos);

        if (height == null) {
            System.err.println(
                "ElevatorSubsystem.Position." + pos.name() + " is not defined in 'positions' map"
            );
            return new Noop();
        }

        return c_gotoHeight(height);
    }
}
