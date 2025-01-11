package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase {

    public final CANTalonFX motor;

    public SingleMotorSubsystem(CANTalonFX motor) {
        this.motor = motor;
        this.motor.setIdleMode(IdleMode.kBrake);
    }

    public Command c_setVoltage(int voltage) {
        return this.run(() -> {
                this.motor.setVoltage(voltage);
            });
    }

    public Command c_forward() {
        return this.c_setVoltage(1);
    }

    public Command c_backward() {
        return this.c_setVoltage(-1);
    }

    public Command c_stop() {
        return this.c_setVoltage(0);
    }
}
