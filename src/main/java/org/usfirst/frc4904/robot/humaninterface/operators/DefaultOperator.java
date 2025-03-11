package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.OrchestraSubsystem;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem;
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

        joystick.button3.onTrue(RobotMap.Component.vision.c_align(VisionSubsystem.TagGroup.ANY, 0));
        joystick.button5.onTrue(RobotMap.Component.vision.c_align(VisionSubsystem.TagGroup.ANY, -1));
        joystick.button6.onTrue(RobotMap.Component.vision.c_align(VisionSubsystem.TagGroup.ANY, 1));
        joystick.button3.onFalse(RobotMap.Component.vision.c_stop());
        joystick.button5.onFalse(RobotMap.Component.vision.c_stop());
        joystick.button6.onFalse(RobotMap.Component.vision.c_stop());

        // orchestra
        joystick.button7.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "delfino",
                2,
                RobotMap.Component.flDrive,
                RobotMap.Component.frDrive
            )
        );
        joystick.button8.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "circus",
                2,
                RobotMap.Component.flDrive,
                RobotMap.Component.frDrive
            )
        );
        joystick.button9.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "coconutNyoom",
                4,
                RobotMap.Component.flDrive,
                RobotMap.Component.frDrive,
                RobotMap.Component.blDrive,
                RobotMap.Component.brDrive
            )
        );
        joystick.button10.onTrue(
            OrchestraSubsystem.c_loadAndPlaySong(
                "PortalRadio",
                4,
                RobotMap.Component.flDrive,
                RobotMap.Component.frDrive,
                RobotMap.Component.blDrive,
                RobotMap.Component.brDrive
            )
        );

        joystick.button12.onTrue(new InstantCommand(OrchestraSubsystem::stopAll));
    }
}
