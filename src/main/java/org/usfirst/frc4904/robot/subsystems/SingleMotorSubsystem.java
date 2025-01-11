package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase {

    public final CANTalonFX motor;

    public final int forwardVoltage;
    public final int backwardVoltage;

    public SingleMotorSubsystem(CANTalonFX motor, int voltage) {
        this(motor, voltage, voltage);
    }

    // BOTH VOLTAGES SHOULD BE POSITIVE - 'backwardVoltage' is negated later
    public SingleMotorSubsystem(CANTalonFX motor, int forwardVoltage, int backwardVoltage) {
        this.motor = motor;
        this.motor.setNeutralMode(NeutralModeValue.Brake);


        this.forwardVoltage = forwardVoltage;
        this.backwardVoltage = backwardVoltage;
    }

    public Command c_setVoltage(int voltage) {
        return this.run(() -> {
                this.motor.setVoltage(voltage);
            });
    }

    public Command c_forward() {
        return this.c_setVoltage(this.forwardVoltage);
    }

    public Command c_backward() {
        return this.c_setVoltage(-this.backwardVoltage);
    }

    public Command c_stop() {
        return this.c_setVoltage(0);
    }
}
