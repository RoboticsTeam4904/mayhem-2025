/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
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
        public static Supplier<Command> COMMAND = Auton::c_straight;
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
    }

    @Override
    public void teleopExecute() {
        SmartDashboard.putBoolean(
            "button",
            RobotMap.HumanInput.Driver.turnJoystick.button1.getAsBoolean()
        );

        SmartDashboard.putNumber(
            "max angular velocity",
            RobotMap.Component.chassis.swerveDrive.getMaximumChassisAngularVelocity()
        );

        RobotMap.Component.vision.periodic();

        // //various logging can go here
        // //TODO: getAbsolutePosition() MIGHT NOT WORK OR BE IN RIGHT UNITS!
        // SmartDashboard.putNumber("FL angle-1", Component.flTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("FL angle-2 (currentyl using 2)", RobotMap.Component.flModule.getAbsoluteAngle());
        //
        // SmartDashboard.putNumber("FR angle-1", Component.flTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("FR angle-2", Component.frModule.getAbsoluteAngle());
        // SmartDashboard.putNumber("BL angle-1", Component.blTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("BL angle-2", Component.blModule.getAbsoluteAngle());
        // SmartDashboard.putNumber("BR angle-1", Component.brTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("BR angle-2", Component.brModule.getAbsoluteAngle());
        //
        // var moduleTargets = Component.chassis.kinematics.toSwerveModuleStates(new ChassisSpeeds(driver.getX(), driver.getY(), driver.getTurnSpeed()));
        // for (var c : moduleTargets){
        //     SmartDashboard.putNumber("placeholder", c.angle.getDegrees());
        // }
        // SmartDashboard.putNumber("FL speed", Component.flDrive.get());
        // SmartDashboard.putNumber("FR speed", Component.frDrive.get());
        // SmartDashboard.putNumber("BL speed", Component.blDrive.get());
        // SmartDashboard.putNumber("BR speed", Component.brDrive.get());
        //
        // SmartDashboard.putNumber("driver X ", driver.getX());
        // SmartDashboard.putNumber("driver Y ", driver.getY());
        // SmartDashboard.putNumber("driver Z ", driver.getTurnSpeed());
        //
        // //navx gyro readings
        // SmartDashboard.putNumber("navx angle", Component.navx.getAngle());
        // SmartDashboard.putBoolean("navx calibrating", Component.navx.isCalibrating());
        // SmartDashboard.putBoolean("navx connected", Component.navx.isConnected());
    }

    @Override
    public void autonomousInitialize() {
        if (!AutonConfig.ENABLED) return;

        AutonConfig.COMMAND.get().schedule();
    }

    @Override
    public void autonomousExecute() {
        // logging can go here
    }

    @Override
    public void disabledInitialize() {
        RobotMap.Component.vision.stopPositioning("Robot disabled");

        Component.elevatorMotorOne.setBrakeOnNeutral();
        Component.elevatorMotorTwo.setBrakeOnNeutral();
    }

    @Override
    public void disabledExecute() {}

    @Override
    public void testInitialize() {
        //do things like setting neutral or brake mode on the mechanism or wheels here
        Component.elevatorMotorOne.setCoastOnNeutral();
        Component.elevatorMotorTwo.setCoastOnNeutral();
    }

    @Override
    public void testExecute() {
        // //coast drive
        // Component.flDrive.stopMotor();
        // Component.frDrive.stopMotor();
        // Component.blDrive.stopMotor();
        // Component.brDrive.stopMotor();
        //
        // Component.flDrive.setNeutralMode(NeutralModeValue.Coast);
        // Component.frDrive.setNeutralMode(NeutralModeValue.Coast);
        // Component.blDrive.setNeutralMode(NeutralModeValue.Coast);
        // Component.brDrive.setNeutralMode(NeutralModeValue.Coast);
        //
        // //coast turn
        // Component.flTurn.neutralOutput();
        // Component.frTurn.neutralOutput();
        // Component.blTurn.neutralOutput();
        // Component.brTurn.neutralOutput();
        //
        // Component.flTurn.setCoastOnNeutral();
        // Component.frTurn.setCoastOnNeutral();
        // Component.blTurn.setCoastOnNeutral();
        // Component.brTurn.setCoastOnNeutral();
        //
        // //various logging can go here
        // //TODO: getAbsolutePosition() MIGHT NOT WORK OR BE IN RIGHT UNITS!
        // SmartDashboard.putNumber("FL angle-1", Component.flTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("FL angle-2 (currently using 2)", Component.flModule.getAbsoluteAngle());
        //
        // SmartDashboard.putNumber("FR angle-1", Component.frTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("FR angle-2", Component.frModule.getAbsoluteAngle());
        // SmartDashboard.putNumber("BL angle-1", Component.blTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("BL angle-2", Component.blModule.getAbsoluteAngle());
        // SmartDashboard.putNumber("BR angle-1", Component.brTurnEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("BR angle-2", Component.brModule.getAbsoluteAngle());
        //
        // var moduleTargets = Component.chassis.kinematics.toSwerveModuleStates(new ChassisSpeeds(driver.getX(), driver.getY(), driver.getTurnSpeed()));
        // for (var c : moduleTargets){
        //     SmartDashboard.putNumber(c.toString(), c.angle.getDegrees());
        // }
        // SmartDashboard.putNumber("FL speed", Component.flDrive.get());
        // SmartDashboard.putNumber("FR speed", Component.frDrive.get());
        // SmartDashboard.putNumber("BL speed", Component.blDrive.get());
        // SmartDashboard.putNumber("BR speed", Component.brDrive.get());
        //
        // SmartDashboard.putNumber("driver X ", driver.getX());
        // SmartDashboard.putNumber("driver Y ", driver.getY());
        // SmartDashboard.putNumber("driver Z ", driver.getTurnSpeed());
        //
        // //navx gyro readings
        // SmartDashboard.putNumber("navx angle", Component.navx.getAngle());
    }

    @Override
    public void alwaysExecute() {
        // logging stuff can go here
        if (Component.elevator != null) {
            System.out.println("ELEVATOR ENCODER: " + Component.elevator.encoder.get());
        }
    }
}
