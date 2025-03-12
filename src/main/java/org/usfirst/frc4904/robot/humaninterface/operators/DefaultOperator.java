package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.OrchestraSubsystem;
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

        joystick.button3.whileTrue(Component.vision.c_align(TagGroup.REEF, 0));
        joystick.button5.whileTrue(Component.vision.c_align(TagGroup.REEF, -1));
        joystick.button6.whileTrue(Component.vision.c_align(TagGroup.REEF, 1));

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
    }
}
