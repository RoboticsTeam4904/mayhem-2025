
package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

public class ElevatorSubsystem extends SubsystemBase {

    public final CANTalonFX motorOne;
    public final CANTalonFX motorTwo;

    public ElevatorSubsystem(CANTalonFX motorOne, CANTalonFX motorTwo)
    {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
    }
    public Command c_ElevatorDirectionA() {
        return this.run(() -> {
                this.motorOne.setVoltage(1);
                this.motorTwo.setVoltage(-1);
            });
    }
    public Command c_ElevatorDirectionB() {
        return this.run(() -> {
                this.motorOne.setVoltage(-1);
                this.motorTwo.setVoltage(1);
            });
    }
}