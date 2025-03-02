package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class RunIf extends RunIfElse {

    /**
     * Run a command based on a conditional callback. For example, if you only want
     * to shoot if a shooter is ready (based on its isReady() function),
     * use: {@code new RunIf(new Shoot(), shooter::isReady)}
     * <p>
     * Conditions are AND-ed together
     *
     * @param command          The command to be run if the condition is met
     * @param booleanSuppliers A variable number of condition functions
     */
    public RunIf(Command command, BooleanSupplier... booleanSuppliers) {
        super(command, new NoOp(), booleanSuppliers);
    }
}
