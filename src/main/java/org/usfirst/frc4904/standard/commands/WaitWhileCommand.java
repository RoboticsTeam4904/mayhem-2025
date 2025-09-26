package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

public class WaitWhileCommand extends WaitUntilCommand {

    public WaitWhileCommand(BooleanSupplier condition) {
        super(() -> !condition.getAsBoolean());
    }
}
