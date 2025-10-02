package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem.TagGroup;
import org.usfirst.frc4904.standard.humaninput.Operator;

public class AnnaOperator extends Operator {

    public AnnaOperator() {
        super("AnnaOperator");
    }

    public AnnaOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        var joystick = RobotMap.HumanInput.Operator.joystick;
        var xyJoystick = RobotMap.HumanInput.Driver.xyJoystick;
        var turnJoystick = RobotMap.HumanInput.Driver.turnJoystick;

        /// ELEVATOR SETPOINTS
        joystick.button7.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.INTAKE));
        joystick.button8.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L2));
        joystick.button9.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L3));

        /// INTAKE
        joystick.button11.onTrue(Component.elevator.c_intakeRaw());

        /// MANUAL RAMP CONTROL
        joystick.button3.onTrue(Component.ramp.c_forward());
        joystick.button5.onTrue(Component.ramp.c_backward());
        joystick.button3.onFalse(Component.ramp.c_stop());
        joystick.button5.onFalse(Component.ramp.c_stop());

        /// MANUAL OUTTAKE CONTROL
        joystick.button4.onTrue(Component.outtake.c_backward());
        joystick.button6.onTrue(Component.outtake.c_forward());
        joystick.button4.onFalse(Component.outtake.c_stop());
        joystick.button6.onFalse(Component.outtake.c_stop());

        /// VISION
        turnJoystick.button1.whileTrue(Component.vision.c_align(TagGroup.ANY, -1));
        turnJoystick.button2.whileTrue(Component.vision.c_align(TagGroup.ANY, 1));

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(new InstantCommand(() -> Component.chassis.resetOdometry(Pose2d.kZero)));

        /// ELEVATOR ENCODER RESETTING
        joystick.button10.onTrue(Component.elevator.c_forceDown());
        joystick.button10.onFalse(Component.elevator.c_resetEncoder());
    }
}
