package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem.TagGroup;
import org.usfirst.frc4904.standard.humaninput.Operator;

public class AyushOperator extends Operator {

    public AyushOperator() {
        super("AyushOperator");
    }

    public AyushOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        var joystick = RobotMap.HumanInput.Operator.joystick;
        var xyJoystick = RobotMap.HumanInput.Driver.xyJoystick;
        var turnJoystick = RobotMap.HumanInput.Driver.turnJoystick;

        /// ELEVATOR SETPOINTS
        joystick.button11.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.INTAKE));
        joystick.button9.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L2));
        joystick.button7.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L3));

        /// MANUAL ELEVATOR CONTROL
        // see Robot.teleopExecute()
        // joystick.button11.onTrue(Component.elevator.c_backward());
        // joystick.button12.onTrue(Component.elevator.c_forward());
        // joystick.button11.onFalse(Component.elevator.c_stop());
        // joystick.button12.onFalse(Component.elevator.c_stop());

        if (true) {
            /// INTAKE
            joystick.button4.onTrue(Component.elevator.c_intakeRaw());
            /// RAMP OUTTAKE
            //joystick.button6.onTrue(Component.elevator.c_rampOuttakeRaw());

            /// MANUAL RAMP CONTROL
            joystick.button5.onTrue(Component.ramp.c_forward());
            joystick.button3.onTrue(Component.ramp.c_backward());
            joystick.button5.onFalse(Component.ramp.c_stop());
            joystick.button3.onFalse(Component.ramp.c_stop());

            /// MANUAL OUTTAKE CONTROL
            joystick.button6.onTrue(Component.outtake.c_forward());
            joystick.button3.onTrue(Component.outtake.c_backward());
            joystick.button6.onFalse(Component.outtake.c_stop());
            joystick.button3.onFalse(Component.outtake.c_stop());
       }

        /// VISION
        turnJoystick.button1.whileTrue(Component.vision.c_align(TagGroup.ANY, -1));
        turnJoystick.button2.whileTrue(Component.vision.c_align(TagGroup.ANY, 1));

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(new InstantCommand(() -> Component.chassis.resetOdometry(Pose2d.kZero)));

        /// ELEVATOR ENCODER RESETTING
        var elevatorEncoderCommand = new InstantCommand(() -> Component.elevator.setVoltage(-3, true));
        elevatorEncoderCommand.addRequirements(Component.elevator);
        joystick.button8.onTrue(elevatorEncoderCommand);
        joystick.button8.onFalse(new InstantCommand(() -> {
            Component.elevator.setVoltage(0);
            Component.elevatorEncoder.reset();
        }));

        /// ORCHESTRA
        /*
        joystick.button7.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "delfino",
                2,
                Component.flDrive,
                Component.frDrive
            )
        );
        joystick.button7.onFalse(new InstantCommand(OrchestraSubsystem::stopAll));

        joystick.button8.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "circus",
                2,
                Component.flDrive,
                Component.frDrive
            )
        );
        joystick.button8.onFalse(new InstantCommand(OrchestraSubsystem::stopAll));

        joystick.button9.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "coconutNyoom",
                4,
                Component.flDrive,
                Component.frDrive,
                Component.blDrive,
                Component.brDrive
            )
        );
        joystick.button9.onFalse(new InstantCommand(OrchestraSubsystem::stopAll));

        joystick.button10.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "PortalRadio",
                4,
                Component.flDrive,
                Component.frDrive,
                Component.blDrive,
                Component.brDrive
            )
        );
        joystick.button10.onFalse(new InstantCommand(OrchestraSubsystem::stopAll));

        joystick.button12.onTrue(new InstantCommand(OrchestraSubsystem::stopAll));
        */

    }
}
