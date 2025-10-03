package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem.Position;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem.TagGroup;
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
        var joystick = HumanInput.Operator.joystick;
        var xyJoystick = HumanInput.Driver.xyJoystick;
        var turnJoystick = HumanInput.Driver.turnJoystick;

        /// ELEVATOR SETPOINTS
        joystick.button11.onTrue(Component.elevator.c_gotoPosition(Position.INTAKE));
        joystick.button9.onTrue(Component.elevator.c_gotoPosition(Position.L2));
        joystick.button7.onTrue(Component.elevator.c_gotoPosition(Position.L3));

        /// MANUAL RAMP CONTROL
        joystick.button3.onTrue(Component.ramp.c_forward());
        joystick.button5.onTrue(Component.ramp.c_backward());
        joystick.button3.onFalse(Component.ramp.c_stop());
        joystick.button5.onFalse(Component.ramp.c_stop());

        /// MANUAL OUTTAKE CONTROL
        joystick.button4.onTrue(Component.elevator.c_intake());
        joystick.button6.onTrue(Component.outtake.c_forward());
        joystick.button6.onFalse(Component.outtake.c_stop());

        /// VISION
        turnJoystick.button1.whileTrue(Component.vision.c_align(TagGroup.ANY, -1));
        turnJoystick.button2.whileTrue(Component.vision.c_align(TagGroup.ANY, 1));

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(c_resetOdometry());

        /// ELEVATOR ENCODER RESETTING
        joystick.button8.whileTrue(c_manualElevatorZero());
    }
}
