package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class RunUnless extends RunIfElse {

    /**
     * Run a command based on a conditional callback. For example, if you only want
     * to shoot if a shooter is safe (based on its isUnsafe() function),
     * use: {@code new RunIf(new Shoot(), shooter::isUnsafe)}
     * <p>
     * Conditions are AND-ed together (if ANY are false, the command will run)
     *
     * @param command          The command to be run if the condition is NOT met
     * @param booleanSuppliers A condition function using Java 8's colon syntax
     */
    public RunUnless(String name, Command command, BooleanSupplier... booleanSuppliers) {
        super(name, new NoOp(), command, booleanSuppliers);
    }

    public RunUnless(Command command, BooleanSupplier... booleanSuppliers) {
        super(new NoOp(), command, booleanSuppliers);
    }
}
