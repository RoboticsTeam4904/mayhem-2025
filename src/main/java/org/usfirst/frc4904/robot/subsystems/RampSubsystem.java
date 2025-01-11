package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampSubsystem extends SubsystemBase{
    public final CANTalonFX rampMotor;
    public RampSubsystem(CANTalonFX rampMotor){
        this.rampMotor = rampMotor;
        this.rampMotor.setIdleMode(IdleMode.kBrake);

    }
    public Command c_rampIn(){
        var cmd = this.run(() -> {
            this.rampMotor.setVoltage(1);
        });
        return cmd;
    }
    public Command c_rampOut(){
        var cmd = this.run(() -> {
            this.rampMotor.setVoltage(-1);
        });
        return cmd;
    }
    public Command c_rampStop(){
        var cmd = this.run(() -> {
            this.rampMotor.setVoltage(0);
        });
        return cmd;
    }
}