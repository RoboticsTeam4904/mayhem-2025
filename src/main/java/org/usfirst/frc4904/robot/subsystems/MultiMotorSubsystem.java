package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

public class MultiMotorSubsystem extends SubsystemBase {

    public final CANTalonFX[] motors;
    public final double[] relativeVoltages;

    public final double forwardVoltage;
    public final double backwardVoltage;

    /**
     * Control multiple motors with one subsystem. For example, to have two motors where the second one is inverted, use:
     * <code>new MultiMotorSubsystem(new CANTalonFX[]{motor1, motor2}, new int[]{1, -1}, voltage)</code>
     *
     * @param motors the motors that the subsystem controls
     * @param relativeVoltages multiplied by forward/backwardVoltage for each motor - use to invert some motors, for example
     * @param voltage voltage when motors are running forwards
     */
    public MultiMotorSubsystem(CANTalonFX[] motors, double[] relativeVoltages, double voltage) {
        this(motors, relativeVoltages, voltage, voltage);
    }

    /**
     * Control multiple motors with one subsystem. For example, to have two motors where the second one is inverted, use:
     * <code>new MultiMotorSubsystem(new CANTalonFX[]{motor1, motor2}, new int[]{1, -1}, voltage)</code>
     *
     * @param motors the motors that the subsystem controls
     * @param relativeVoltages multiplied by forward/backwardVoltage for each motor - use to invert some motors, for example
     * @param forwardVoltage voltage when motors are running forwards
     * @param backwardVoltage voltage when motors are running backwards - should be POSITIVE (is negated later)
     */
    public MultiMotorSubsystem(
        CANTalonFX[] motors,
        double[] relativeVoltages,
        double forwardVoltage,
        double backwardVoltage
    ) {
        this.motors = motors;
        this.relativeVoltages = relativeVoltages;
        this.forwardVoltage = forwardVoltage;
        this.backwardVoltage = backwardVoltage;

        for (CANTalonFX motor : motors) {
            motor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    public Command c_setVoltage(double voltage) {
        return this.run(() -> {
                for (int i = 0; i < motors.length; i++) {
                    motors[i].setVoltage(voltage * relativeVoltages[i]);
                }
            });
    }

    public Command c_forward() {
        return this.c_setVoltage(forwardVoltage);
    }

    public Command c_backward() {
        return this.c_setVoltage(-backwardVoltage);
    }

    public Command c_stop() {
        return this.c_setVoltage(0);
    }
}
