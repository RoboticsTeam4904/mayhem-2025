package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterSubsystem extends MotorSubsystem {
    public ShooterSubsystem(CustomCANSparkMax jimmy) {  //Jimmy is left, Johnny is right
        super(new SmartMotorController[] { jimmy }, 12.0);
    }

    public Command c_shoot() {
        return c_forward();
    }

    public Command c_stopShoot() {
        //cancel early
        return c_stop();
    }

    public Command c_timeshoot() {
        return c_shoot().withTimeout(5).finallyDo(this::stop);
    }

    public Command c_usedToBeTrapdoor() {
        return new SequentialCommandGroup(
            c_backward().withTimeout(1),
            c_timeshoot()
        );
    }
}
