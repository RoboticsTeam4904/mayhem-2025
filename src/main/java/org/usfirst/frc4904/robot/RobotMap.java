package org.usfirst.frc4904.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Filesystem;
import org.photonvision.PhotonCamera;
import org.usfirst.frc4904.robot.humaninterface.HumanInterfaceConfig;
import org.usfirst.frc4904.robot.subsystems.*;
import org.usfirst.frc4904.standard.custom.CustomEncoder;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;

import java.io.File;

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

        }

        public static class PWM {

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

        // motors
        // TODO
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
            3
        );

        // TODO 2025: initialize robot parts

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
