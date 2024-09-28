package org.usfirst.frc4904.robot.humaninterface.operators;

// import org.usfirst.frc4904.robot.RobotMap;
// import org.usfirst.frc4904.standard.commands.CreateAndDisown;
import org.usfirst.frc4904.standard.humaninput.Operator;

// import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DefaultOperator extends Operator {

    // private boolean justHeldHighCone = false;

    public DefaultOperator() {
        super("DefaultOperator");
    }

    public DefaultOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        // var joystick = RobotMap.HumanInput.Operator.joystick;

        // manual extension and retraction
        // joystick.button3.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> -0.45));
        // joystick.button3.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));
        // joystick.button5.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0.45));
        // joystick.button5.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));

        // // Intake
        // // FIXME: use nameCommand to make it cleaner with expresions (no varibales)
        // var zeroIntake = RobotMap.Component.intake.c_holdVoltage(0);
        // var runOuttake = RobotMap.Component.intake.c_holdVoltage(3);

        // // intake
        // joystick.button2.onTrue(RobotMap.Component.intake.c_startIntake());
        // joystick.button2.onFalse(RobotMap.Component.intake.c_holdItem());

        // // outtake
        // joystick.button1.onTrue(runOuttake);
        // joystick.button1.onFalse(justHeldHighCone ? zeroIntake.andThen(new CreateAndDisown(() -> (RobotMap.Component.arm.c_posReturnToHomeUp().andThen(() -> {justHeldHighCone = false;})))) : zeroIntake);

        // // position + place cube
        // joystick.button7 .onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_shootCubes(3)));
        // joystick.button9 .onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_shootCubes(2)));

        // // position cone
        // joystick.button8 .onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_shootCones(3, true).andThen(() -> { justHeldHighCone = true; })));
        // joystick.button8.onFalse(new InstantCommand(() -> { justHeldHighCone = true; }));

        // joystick.button10.onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_shootCones(2)));

        // // intake positions
        // joystick.button6 .onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posIntakeShelf(null)));
        // joystick.button4 .onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posIntakeFloor(null)));

        // // stow positions
        // joystick.button11.onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posReturnToHomeDown(null)));
        // joystick.button12.onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posReturnToHomeUp(null)));

        // // intake positions
        // joystick.button6 .onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posIntakeShelf(() -> RobotMap.Component.intake.c_startIntake())));
        // joystick.button4 .onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posIntakeFloor(() -> RobotMap.Component.intake.c_startIntake())));

        // // stow positions
        // joystick.button11.onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posReturnToHomeDown(() -> RobotMap.Component.intake.c_holdItem())));
        // joystick.button12.onTrue(new CreateAndDisown(() -> RobotMap.Component.arm.c_posReturnToHomeUp(() -> RobotMap.Component.intake.c_holdItem())));
    }
}
