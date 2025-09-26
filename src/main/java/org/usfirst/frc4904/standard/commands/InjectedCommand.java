package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class InjectedCommand extends Command {

    private final Command previous;

    public InjectedCommand(String name, Command previous) {
        super();
        setName(name);
        this.previous = previous;

        CommandScheduler.getInstance().registerComposedCommands(previous);
    }

    public InjectedCommand(Command previous) {
        this("Injected(" + previous.getName() + ")", previous);
    }

    @Override
    public final void initialize() {
        if (previous != null && previous.isScheduled()) {
            previous.cancel();
        }
        onInitialize();
    }

    @Override
    public final void end(boolean interrupted) {
        onEnd(interrupted);
        if (previous != null && !previous.isScheduled()) {
            previous.schedule();
        }
    }

    protected abstract void onInitialize();

    protected abstract void onInterrupted();

    protected abstract void onEnd(boolean interrupted);
}
