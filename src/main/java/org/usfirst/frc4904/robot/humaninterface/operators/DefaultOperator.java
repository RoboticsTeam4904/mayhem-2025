package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.humaninput.Operator;

public class DefaultOperator extends Operator {

    public DefaultOperator() {
        super("DefaultOperator");
    }

    public DefaultOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        var joystick = RobotMap.HumanInput.Operator.joystick;
        var xyJoystick = RobotMap.HumanInput.Driver.xyJoystick;
        var turnJoystick = RobotMap.HumanInput.Driver.turnJoystick;

        // TODO: bind commands

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(new InstantCommand(() -> Component.chassis.resetOdometry(Pose2d.kZero)));

    }
}
