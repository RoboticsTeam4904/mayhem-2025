package org.usfirst.frc4904.standard.custom.motioncontrollers;

@FunctionalInterface
public interface ezFeedForward {
    double calculate(double setpoint, double setpoint_dt);
}
