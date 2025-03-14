package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
// import org.usfirst.frc4904.robot.subsystems.OrchestraSubsystem;
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
        var joystick = RobotMap.HumanInput.Operator.joystick;
        var turnJoystick = RobotMap.HumanInput.Driver.turnJoystick;

        // TODO IMPORTANT: change from "5" to TagGroup.REEF
        turnJoystick.button1.whileTrue(Component.vision.c_align(5, -1));
        turnJoystick.button2.whileTrue(Component.vision.c_align(5, 1));

        joystick.button3.whileTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.INTAKE));
        joystick.button5.whileTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L2));
        joystick.button6.whileTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L3));

        // joystick.button7.onTrue(Component.elevator.c_intake());
        // joystick.button8.onTrue(Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L1));
        joystick.button7.onTrue(Component.elevator.c_intakeRaw());
        joystick.button8.onTrue(Component.elevator.c_outtakeRaw());

        // OUTTAKE
        joystick.button9.onTrue(Component.elevator.c_rampOuttakeRaw());
        joystick.button9.onFalse(Component.elevator.c_stop());

        // TODO IMPORTANT: bind a button to climber

        // ELEVATOR
        joystick.button11.onTrue(Component.elevator.c_forward());
        joystick.button12.onTrue(Component.elevator.c_backward());
        joystick.button11.onFalse(Component.elevator.c_stop());
        joystick.button12.onFalse(Component.elevator.c_stop());

        /*
        // orchestra
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
