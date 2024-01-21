/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
//import java.util.function.Supplier;

import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
//import org.usfirst.frc4904.standard.CommandRobotBase;
// import org.usfirst.frc4904.standard.custom.CommandSendableChooser;
import org.usfirst.frc4904.standard.humaninput.Driver;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;

// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.usfirst.frc4904.robot.Utils.nameCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;

public class Robot extends CommandRobotBase {
    // private final RobotMap map = new RobotMap();
    // private final RobotContainer2 donttouchme = new RobotContainer2(RobotMap.Component.frontLeftWheelTalon, RobotMap.Component.backLeftWheelTalon, RobotMap.Component.frontRightWheelTalon, RobotMap.Component.backRightWheelTalon, RobotMap.Component.navx);
    // private SendableChooser<Supplier<Command>> autonomousCommand = new SendableChooser<Supplier<Command>>();

    private final Driver driver = new SwerveGain();
    private final org.usfirst.frc4904.standard.humaninput.Operator operator = new DefaultOperator();

    protected double scaleGain(double input, double gain, double exp) {
		return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
	}

    public Robot() {
        super();
        //can set deafault auton command here
        }

    @Override
    public void initialize() {
    }

    @Override
    public void teleopInitialize() {

        driver.bindCommands();
        operator.bindCommands();

        final double TURN_MULTIPLIER = 2;
        RobotMap.Component.chassis.setDefaultCommand(
            nameCommand("chassis - Teleop_Default - c_swerveDrive", 
                RobotMap.Component.chassis.c_drive(
                    () -> {
                        return new ChassisSpeeds(driver.getX(), driver.getY(), driver.getTurnSpeed());
                    }, true)
                ));
    }

    @Override
    public void teleopExecute() {
        //various logging can go here
        //TODO: getAbsolutePosition() MIGHT NOT WORK OR BE IN RIGHT UNITS!
        SmartDashboard.putNumber("FL angle", RobotMap.Component.FLturnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("FR angle", RobotMap.Component.FRturnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("BL angle", RobotMap.Component.BLturnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("BR angle", RobotMap.Component.BRturnEncoder.getAbsolutePosition());

        SmartDashboard.putNumber("FL speed", RobotMap.Component.FLdrive.get());
        SmartDashboard.putNumber("FR speed", RobotMap.Component.FRdrive.get());
        SmartDashboard.putNumber("BL speed", RobotMap.Component.BLdrive.get());
        SmartDashboard.putNumber("BR speed", RobotMap.Component.BRdrive.get());

        SmartDashboard.putNumber("driver X ", driver.getX());
        SmartDashboard.putNumber("driver Y ", driver.getY());
        SmartDashboard.putNumber("driver Z ", driver.getTurnSpeed());

    }

    @Override
    public void autonomousInitialize() {
        //start autons here
    }

    @Override
    public void autonomousExecute() {
        //logging can go here
    }

    @Override
    public void disabledInitialize() {
        //do things like setting brake mode here
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
        //do things like setting neutral or brake mode on the mechanism or wheels here
    }
    
    @Override
    public void testExecute() {
    }

    @Override
    public void alwaysExecute() {
        // logging stuff can go here
    }
}




    
