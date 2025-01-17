package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

public class ElevatorSubsystem extends SubsystemBase {

    public static final double voltage = 1;

    public final CANTalonFX motorOne;
    public final CANTalonFX motorTwo;

    public ElevatorSubsystem(CANTalonFX motorOne, CANTalonFX motorTwo) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
    }

    public Command c_forwards() {
        return this.run(() -> {
                this.motorOne.setVoltage(voltage);
                this.motorTwo.setVoltage(-voltage);
            });
    }

    public Command c_backwards() {
        return this.run(() -> {
                this.motorOne.setVoltage(-voltage);
                this.motorTwo.setVoltage(voltage);
            });
    }

    public Command c_stop() {
        return this.run(() -> {
                this.motorOne.setVoltage(0);
                this.motorTwo.setVoltage(0);
            });
    }
}
