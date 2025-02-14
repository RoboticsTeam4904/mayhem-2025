package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class SingleMotorSubsystem extends SubsystemBase {

    public final SmartMotorController motor;

    public final double forwardVoltage;
    public final double backwardVoltage;

    public SingleMotorSubsystem(SmartMotorController motor, double voltage) {
        this(motor, voltage, voltage);
    }

    // BOTH VOLTAGES SHOULD BE POSITIVE - 'backwardVoltage' is negated later
    public SingleMotorSubsystem(SmartMotorController motor, double forwardVoltage, double backwardVoltage) {
        this.motor = motor;
        this.motor.setBrakeOnNeutral();

        this.forwardVoltage = forwardVoltage;
        this.backwardVoltage = backwardVoltage;
    }

    public Command c_holdVoltage(double voltage) {
        return this.run(() -> this.motor.setVoltage(voltage));
    }

    public Command c_forward() {
        return this.c_holdVoltage(this.forwardVoltage);
    }

    public Command c_backward() {
        return this.c_holdVoltage(-this.backwardVoltage);
    }

    public Command c_stop() {
        return this.c_holdVoltage(0);
    }
}
