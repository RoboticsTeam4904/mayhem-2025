package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase {

    public final CANTalonFX motor;

    public final int forwardVoltage;
    public final int backwardVoltage;

    public SingleMotorSubsystem(CANTalonFX motor, int voltage) {
        this.SingleMotorSubsystem(motor, voltage, voltage);
    }

    // BOTH VOLTAGES SHOULD BE POSITIVE - 'backwardVoltage' is negated later
    public SingleMotorSubsystem(CANTalonFX motor, int forwardVoltage, int backwardVoltage) {
        this.motor = motor;
        this.motor.setIdleMode(IdleMode.kBrake);

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
