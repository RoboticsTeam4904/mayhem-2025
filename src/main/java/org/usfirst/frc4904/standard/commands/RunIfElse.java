package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import org.usfirst.frc4904.robot.CmdUtils;
import org.usfirst.frc4904.standard.Util;

import java.util.function.BooleanSupplier;

public class RunIfElse extends ConditionalCommand {
    /**
     * Similar to {@link RunIf}.
     * <p>
     * Conditions are AND-ed together:
     * <ul>
     *   <li> onTrue will only run if ALL conditions are true
     *   <li> onFalse will run if ANY conditions are false
     * </ul>
     */
    public RunIfElse(Command onTrue, Command onFalse, BooleanSupplier... conditions) {
        super(
            CmdUtils.nonNull(onTrue),
            CmdUtils.nonNull(onFalse),
            Util.all(conditions)
        );
    }
}
