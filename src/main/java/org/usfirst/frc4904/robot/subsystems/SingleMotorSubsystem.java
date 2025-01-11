package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase{
    public final CANTalonFX motor;
    public SingleMotorSubsystem(CANTalonFX motor){
        this.motor = motor;
        this.motor.setIdleMode(IdleMode.kBrake);

    }
    public Command c_forward(){
        var cmd = this.run(() -> {
            this.motor.setVoltage(1);
        });
        return cmd;
    }
    public Command c_backward(){
        var cmd = this.run(() -> {
            this.motor.setVoltage(-1);
        });
        return cmd;
    }
    public Command c_stop(){
        var cmd = this.run(() -> {
            this.motor.setVoltage(0);
        });
        return cmd;
    }
}