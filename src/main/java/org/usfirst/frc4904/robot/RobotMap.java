package org.usfirst.frc4904.robot;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import org.usfirst.frc4904.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc4904.robot.swerve.SwerveModule;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;

public class RobotMap {

    public static class Port {

        public static class HumanInput {

            public static final int xyJoystickPort = 0;
            public static final int zJoystickPort = 1;

            public static final int joystick = 2;
        }

        public static class CANMotor {

            public static final int WHEEL = -1;
        }

        public static class PWM {

            public static final int ENCODER_FL = 0;
            public static final int ENCODER_FR = 1;
            public static final int ENCODER_BL = 2;
            public static final int ENCODER_BR = 3;
        }
    }

    public static class Component {

        public static CANTalonFX flDrive;
        public static CANTalonFX frDrive;
        public static CANTalonFX blDrive;
        public static CANTalonFX brDrive;

        public static AHRS navx;

        // subsystems
        public static SwerveSubsystem chassis;

        public static ShooterSubsystem shooter;

        public static CustomCANSparkMax wheelMotor;
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
            new SwerveModule(
                new CANTalonFX(1),
                new CustomCANSparkMax(5, MotorType.kBrushless, false),
                new DutyCycleEncoder(Port.PWM.ENCODER_FL),
                new Translation2d(-1, 1)
            ),
            new SwerveModule(
                new CANTalonFX(2),
                new CustomCANSparkMax(6, MotorType.kBrushless, false),
                new DutyCycleEncoder(Port.PWM.ENCODER_FR),
                new Translation2d(1, 1)
            ),
            new SwerveModule(
                new CANTalonFX(3),
                new CustomCANSparkMax(7, MotorType.kBrushless, false),
                new DutyCycleEncoder(Port.PWM.ENCODER_BL),
                new Translation2d(-1, -1)
            ),
            new SwerveModule(
                new CANTalonFX(4),
                new CustomCANSparkMax(8, MotorType.kBrushless, false),
                new DutyCycleEncoder(Port.PWM.ENCODER_BR),
                new Translation2d(1, -1)
            )
        );

        Component.wheelMotor = new CustomCANSparkMax(
            Port.CANMotor.WHEEL,
            SparkLowLevel.MotorType.kBrushless,
            false
        );

        Component.shooter = new ShooterSubsystem(Component.wheelMotor);

        HumanInput.Driver.xyJoystick = new CustomCommandJoystick(
            Port.HumanInput.xyJoystickPort,
            0.0
        );
        HumanInput.Driver.turnJoystick = new CustomCommandJoystick(
            Port.HumanInput.zJoystickPort,
            0.0
        );

        HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
    }
}
