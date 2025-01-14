package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase {
    Orchestra m_orchestra = new Orchestra();
    public CANTalonFX motor;

    public Command playSound(String file, CANTalonFX motor){
        m_orchestra.addInstrument(motor);
        m_orchestra.loadMusic(file);
        return this.run(() -> {
            m_orchestra.play();
        });
    }
}