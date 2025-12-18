/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;

import com.revrobotics.jni.CANSparkJNI;

import java.util.function.Supplier;

public class Robot extends CommandRobotBase {

    public static class AutonConfig {

        /** Whether to run auton at all */
        public static final boolean ENABLED = false;

        /** Whether to flip the path to the other side of the current alliance's field */
        public static final boolean FLIP_SIDE = false;

        /** The auton to run */
        public static Supplier<Command> COMMAND = Auton::c_jankStraight;
    }

    private final SwerveGain driver = new SwerveGain();
    private final DefaultOperator operator = new DefaultOperator();
    private final RobotMap map = new RobotMap();

    protected double scaleGain(double input, double gain, double exp) {
        return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
    }

    public Robot() {
        super();
    }

    @Override
    public void initialize() {}

    @Override
    public void teleopInitialize() {
        driver.bindCommands();
        operator.bindCommands();

        Component.chassis.setDefaultCommand(
            Component.chassis.c_input(driver::getTranslation, driver::getTurnSpeed)
        );

        Component.chassis.resetOdometry();

        // Component.chassis.zero();
    }

    @Override
    public void teleopExecute() {
        // RobotMap.Component.wheelMotor.setVoltage(2);
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
        Component.chassis.setMotorBrake(false);
     }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
        // do things like setting neutral or brake mode on the mechanism or wheels here
    }

    @Override
    public void testExecute() {

    }

    double lastLogTime = 0;

    @Override
    public void alwaysExecute() {
        // logging stuff cannot go here. turn back now
    }
}
