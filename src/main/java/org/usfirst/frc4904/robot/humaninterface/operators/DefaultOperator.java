package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.OrchestraSubsystem;
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

        SequentialCommandGroup command = Commands.runOnce(() -> {
            Transform2d targetPose = new Transform2d(
                1.0, 0.0, new Rotation2d(Math.PI)
            );
            RobotMap.Component.vision.startPositioning(targetPose);
        })
        .andThen(Commands.waitUntil(() -> !RobotMap.Component.vision.isPositioning()));

        command.addRequirements(RobotMap.Component.chassis);

        joystick.button4.onTrue(command);

        //Orchestra Music Init
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
