/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.subsystems.LightSubsystem;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.humaninput.Driver;

import java.util.function.Supplier;

public class Robot extends CommandRobotBase {

    public static class AutonConfig {

        /** Whether to run auton at all */
        public static final boolean ENABLED = true;

        /** Whether to flip the path to the other side of the current alliance's field */
        public static final boolean FLIP_SIDE = false;

        /** The auton to run */
        public static Supplier<Command> COMMAND = Auton::c_jankRightCoral;
    }

    private final Driver driver = new SwerveGain();
    private final DefaultOperator operator = new DefaultOperator();
    private final RobotMap map = new RobotMap();

    protected double scaleGain(double input, double gain, double exp) {
        return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
    }

    public Robot() {
        super();
        //can set default auton command here
    }

    @Override
    public void initialize() {}

    @Override
    public void teleopInitialize() {
        driver.bindCommands();
        operator.bindCommands();
        //Component.elevator.encoder.reset();

        Component.chassis.setDefaultCommand(
            Component.chassis.driveCommand(driver::getY, driver::getX, driver::getTurnSpeed)
        );

        Component.lights.flashColor(LightSubsystem.Color.ENABLED);
    }

    boolean wasControllingElevator = false;

    @Override
    public void teleopExecute() {
        // TODO maybe unnecessary
        Component.vision.periodic();

        double y = RobotMap.HumanInput.Operator.joystick.getY();

        if (Math.abs(y) >= 0.05) {
            wasControllingElevator = true;
            
            var command = new InstantCommand(
                () -> Component.elevator.setVoltage(Math.pow(y, 2) * Math.signum(y) * 12.0)
            );
            command.addRequirements(Component.elevator);
            command.schedule();
        } else if (wasControllingElevator) {
            wasControllingElevator = false;
            Component.elevator.setVoltage(0);
        }
    }

    Timer timer = new Timer();

    @Override
    public void autonomousInitialize() {
        if (!AutonConfig.ENABLED) return;

        Component.navx.reset();

        // timer.reset();
        // timer.start();

        AutonConfig.COMMAND.get().schedule();
    }

    @Override
    public void autonomousExecute() {
<<<<<<< HEAD
        // if (timer.get() < 1.0) {
        //     Component.chassis.drive(               
        //         ChassisSpeeds.fromRobotRelativeSpeeds(
        //             3.0,
        //             0.0,
        //             0.0,
        //             Rotation2d.kZero
        //         )
        //     );
        // } else{
        //     Component.chassis.drive(               
        //         ChassisSpeeds.fromRobotRelativeSpeeds(
        //             0.0,
        //             0.0,
        //             0.0,
        //             Rotation2d.kZero
        //         )
        //     ); 
        // }
=======
        if (timer.get() < 1.0) {
            Component.chassis.drive(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    3.0,
                    0.0,
                    0.0,
                    Rotation2d.kZero
                )
            );
        } else{
            Component.chassis.drive(               
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    0.0,
                    0.0,
                    0.0,
                    Rotation2d.kZero
                )
            );
        }
>>>>>>> 4fdf5ef9be026201228367ae240708cdeaac9892

        // logging can go here
    }

    @Override
    public void disabledInitialize() {
        Component.vision.stopPositioning("Robot disabled", false);

        Component.lights.flashColor(LightSubsystem.Color.DISABLED);

    //     Component.elevatorMotorOne.setBrakeOnNeutral();
    //     Component.elevatorMotorTwo.setBrakeOnNeutral();
     }

    @Override
    public void disabledExecute() {}

    @Override
    public void testInitialize() {
        //do things like setting neutral or brake mode on the mechanism or wheels here
        // Component.elevatorMotorOne.setCoastOnNeutral();
        // Component.elevatorMotorTwo.setCoastOnNeutral();
    }

    @Override
    public void testExecute() {

    }

    double lastLogTime = 0;

    @Override
    public void alwaysExecute() {
        // logging stuff can go here
        // if (Component.elevator != null && Timer.getFPGATimestamp() - lastLogTime > 0.2) {
        //     lastLogTime = Timer.getFPGATimestamp();
        //     System.out.printf("ELEVATOR ENCODER: %.4f%n", Component.elevatorEncoder.get());
        // }
    }
}
