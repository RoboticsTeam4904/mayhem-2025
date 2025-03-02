package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class NoOp extends Command {

    public NoOp() {
        super();
        setName("NoOp");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
