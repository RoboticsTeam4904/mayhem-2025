package org.usfirst.frc4904.robot;

import com.revrobotics.spark.SparkLowLevel;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.AddressableLED;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import java.io.File;

import org.usfirst.frc4904.robot.subsystems.*;
import org.usfirst.frc4904.standard.custom.CustomEncoder;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import org.usfirst.frc4904.robot.humaninterface.HumanInterfaceConfig;

// import org.usfirst.frc4904.standard.LogKitten;

// import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
//import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;

public class RobotMap {

    public static class Port {

        public static class HumanInput {

            public static final int xyJoystickPort = 0;
            public static final int zJoystickPort = 1;

            public static final int joystick = 2;
        }

        public static class CANMotor {

            // MOTOR TIME
            // public static final int FRONT_LEFT_DRIVE = 1;
            // public static final int FRONT_RIGHT_DRIVE = 2;
            // public static final int BACK_LEFT_DRIVE = 3;
            // public static final int BACK_RIGHT_DRIVE = 4;

            public static final int FRONT_LEFT_TURN = 5;
            public static final int FRONT_RIGHT_TURN = 6;
            public static final int BACK_LEFT_TURN = 7;
            public static final int BACK_RIGHT_TURN = 8;

            public static final int RAMP = 10;

            public static final int OUTTAKE_MOTOR_RIGHT = 26;
            public static final int OUTTAKE_MOTOR_LEFT = 27;

            public static final int CLIMBER = 60;

            public static final int ELEVATOR_RIGHT = 15;
            public static final int ELEVATOR_LEFT = 16;
        }

        public static class PWM {

            public static final int ENCODER_FL = 0;
            public static final int ENCODER_FR = 1;
            public static final int ENCODER_BL = 2;
            public static final int ENCODER_BR = 3;

            public static final int ELEVATOR_ENCODER = 0;

            public static final int LED_STRIP = -1; // TODO
        }

        public static class CAN {}

        public static class Pneumatics {}

        public static class Digital {}
    }

    public static class Metrics {

        public static class Chassis {

            public static final double GEAR_RATIO_DRIVE = 5.1; // 5.1:1 gear ratio
            public static final double GEAR_RATIO_TURN = 46.42; // 46.42:1 gear ratio
            public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3); // 3 inch wheels
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(12.241 * 2); // +/- 0.5 inches
            public static final double TRACK_LENGTH_METERS = Units.inchesToMeters(12.259 * 2); //
            public static final double CHASSIS_LENGTH = Units.inchesToMeters(28); // +/- 0.5 inches
            public static final Translation2d CENTER_MASS_OFFSET = new Translation2d(0, 0); // no offset
            public static final double EncoderTicksPerRevolution = 2048;

            //these are allowed maxes rather than max capabilities
            //constants taken from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java
            public static final double MAX_SPEED = 8; //allowed max speed in meters per second
            public static final double MAX_ACCELERATION = 3; //allowed max acceleration in meters per second squared
            public static final double MAX_TRANSLATION_SPEED = 4.8;
            public static final double MAX_TURN_SPEED = 360; //allowed max turn speed in degrees per second
            public static final double MAX_TURN_ACCELERATION = 180; //allowed max turn acceleration in degrees per second squared
        }
    }

    public static class PID {

        public static class Drive {

            // PID constants
            // public static final double kP = 1.5;
            public static final double kP = .04; //TODO: tune, this is from maxswerve repo but seems too low
            public static final double kI = 0; // FIXME: tune
            public static final double kD = 0;
            // feedforward constants
            // pre-sfr on-carpet characterization
            // public static final double kS = 0.025236;
            // public static final double kV = 3.0683;
            // public static final double kA = 0.7358;
            //post sfr characterization
            public static final double kS = .02; //TODO: these are placeholders use sysid to find these
            public static final double kV = 3;
            public static final double kA = .5;
        }

        public static class Turn { //TODO: tune

            // PID constants
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
            // feedforward constants
            public static final double kS = .02; //TODO:these are placeholders use sysid to find these
            public static final double kV = 3;
            public static final double kA = .5;
        }
    }

    public static class Component {

        //TODO: turn motors are NOT falcons and so can't use cantalons
        public static CANTalonFX flDrive;
        // public static CustomCANSparkMax flTurn;
        public static CANTalonFX frDrive;
        // public static CustomCANSparkMax frTurn;
        public static CANTalonFX blDrive;
        // public static CustomCANSparkMax blTurn;
        public static CANTalonFX brDrive;
        // public static CustomCANSparkMax brTurn;

        //encoders are dutycycle encoders, not standard can encoders
        public static DutyCycleEncoder flTurnEncoder;
        public static DutyCycleEncoder frTurnEncoder;
        public static DutyCycleEncoder blTurnEncoder;
        public static DutyCycleEncoder brTurnEncoder;

        public static CustomEncoder elevatorEncoder;

        public static AHRS navx;
        public static SPI serialPort;

        //Subsystems
        public static SwerveSubsystem chassis;
        public static SingleMotorSubsystem ramp;
        public static ElevatorSubsystem elevator;
        public static MultiMotorSubsystem outtake;
        public static SingleMotorSubsystem climber;
        public static VisionSubsystem vision;
        public static LightSubsystem lights;

        //Motor time
        public static CustomCANSparkMax rampMotor;

        public static CustomCANSparkMax outtakeMotorLeft;
        public static CustomCANSparkMax outtakeMotorRight;

        public static CANTalonFX elevatorMotorOne;
        public static CANTalonFX elevatorMotorTwo;

        public static CANTalonFX climberMotor;

        // public static PhotonCamera camera;
        public static PhotonCamera cameraLeft;
        public static PhotonCamera cameraRight;

        public static AddressableLED ledStrip;
    }

    public static class NetworkTables {

        public static NetworkTableInstance instance;

        public static class Odometry {

            public static NetworkTable table;
            public static NetworkTableEntry pose;
            public static NetworkTableEntry accel;
            public static NetworkTableEntry turretAngle;
        }

        public static class Localization {

            public static NetworkTable table;
            public static NetworkTableEntry goalDistance;
            public static NetworkTableEntry goalRelativeAngle;
        }
    }

    public static class Input {}

    public static class HumanInput {

        public static class Driver {

            public static CustomCommandXbox xbox;
            public static CustomCommandJoystick xyJoystick;
            public static CustomCommandJoystick turnJoystick;
        }

        public static class Operator {

            public static CustomCommandJoystick joystick;
        }
    }

    public RobotMap() {
        Component.navx = new AHRS(NavXComType.kMXP_SPI);

        Component.chassis = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve"),
            360,
            0.0473,
            5
        );
        Component.chassis.swerveDrive.setGyroOffset(new Rotation3d(0, 0, 180 - 45));

        // Component.cameraLeft = new PhotonCamera("dauntless-camera-left");
        Component.cameraRight = new PhotonCamera("dauntless-camera");
        Component.vision = new VisionSubsystem(
            new PhotonCamera[] {
                // Component.cameraLeft,
                Component.cameraRight
            },
            new Transform2d[] {
                // new Transform2d(Units.inchesToMeters(-11), Units.inchesToMeters(-10.6), Rotation2d.kZero),
                new Transform2d(Units.inchesToMeters(-8), Units.inchesToMeters(10.6), Rotation2d.kZero)
            }
        );

        Component.rampMotor = new CustomCANSparkMax(
            Port.CANMotor.RAMP,
            SparkLowLevel.MotorType.kBrushless,
            false
        );
        Component.ramp = new SingleMotorSubsystem(Component.rampMotor, -7, -7);

        Component.outtakeMotorLeft = new CustomCANSparkMax(
            Port.CANMotor.OUTTAKE_MOTOR_LEFT,
            SparkLowLevel.MotorType.kBrushless,
            false
        );
        Component.outtakeMotorRight = new CustomCANSparkMax(
            Port.CANMotor.OUTTAKE_MOTOR_RIGHT,
            SparkLowLevel.MotorType.kBrushless,
            false
        );
        Component.outtake = new MultiMotorSubsystem(
            new SmartMotorController[] { Component.outtakeMotorLeft, Component.outtakeMotorRight },
            new double[] { 1, 1 },
            -4
        );

        Component.climberMotor = new CANTalonFX(Port.CANMotor.CLIMBER);
        Component.climber = new SingleMotorSubsystem(Component.climberMotor, 6);

        Component.elevatorMotorOne = new CANTalonFX(Port.CANMotor.ELEVATOR_LEFT);
        Component.elevatorMotorTwo = new CANTalonFX(Port.CANMotor.ELEVATOR_RIGHT);
        Component.elevatorEncoder = new CustomEncoder(new DutyCycleEncoder(Port.PWM.ELEVATOR_ENCODER));
        Component.elevator = new ElevatorSubsystem(
            Component.elevatorMotorOne,
            Component.elevatorMotorTwo,
            Component.elevatorEncoder
        );

        Component.ledStrip = new AddressableLED(Port.PWM.LED_STRIP);
        Component.lights = new LightSubsystem(Component.ledStrip, 10);

        HumanInput.Driver.xyJoystick = new CustomCommandJoystick(
            Port.HumanInput.xyJoystickPort,
            HumanInterfaceConfig.JOYSTICK_DEADZONE
        );
        HumanInput.Driver.turnJoystick = new CustomCommandJoystick(
            Port.HumanInput.zJoystickPort,
            HumanInterfaceConfig.JOYSTICK_DEADZONE
        );

        HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
    }
}
