package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class Utils {

    public static Command nameCommand(String name, Command cmd) {
        cmd.setName(name);
        return cmd;
    }
}
