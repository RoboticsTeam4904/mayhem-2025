package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

public class ElevatorSubsystem extends MultiMotorSubsystem {

    public static double UP_VOLTAGE = 1;
    public static double DOWN_VOLTAGE = 1;

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
