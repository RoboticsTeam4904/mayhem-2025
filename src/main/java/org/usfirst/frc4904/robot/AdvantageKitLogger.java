package org.usfirst.frc4904.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class AdvantageKitLogger {

    public static Logger logger;

    public static void configureDataReceivers() {
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        }
    }

    public static void updateSwerveModuleStates(
        SwerveModuleState[] desiredStates,
        SwerveModuleState[] actualStates,
        double rotation
    ) {
        Logger.recordOutput("Swerve/Display/Target Swerve Module States", desiredStates);
        Logger.recordOutput("Swerve/Display/Actual Swerve Module States", actualStates);
        Logger.recordOutput("Swerve/Display/Rotation", rotation);
    }
}
