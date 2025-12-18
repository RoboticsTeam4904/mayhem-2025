package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DefaultOperator extends Operator {

    public DefaultOperator() {
        super("DefaultOperator");
    }

    public DefaultOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        var opJoystick = RobotMap.HumanInput.Operator.joystick;
        var xyJoystick = RobotMap.HumanInput.Driver.xyJoystick;
        var turnJoystick = RobotMap.HumanInput.Driver.turnJoystick;

        // opJoystick.button11.whileTrue(Component.shooter.c_shoot());
        // opJoystick.button11.onFalse(Component.shooter.c_stopShoot());

        xyJoystick.button1.onTrue(new InstantCommand(() -> Component.chassis.resetOdometry()));
        xyJoystick.button2.onTrue(new InstantCommand(() -> Component.chassis.zero()));
    }
}
