package org.usfirst.frc4904.standard.custom;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CustomDutyCycleEncoder extends DutyCycleEncoder {
    private double resetOffset = 0;

    public CustomDutyCycleEncoder(int channel) {
        super(channel);
        reset();
    }

    public void reset() {
        resetOffset = super.get();
    }

    @Override
    public double get() {
        double value = super.get() - resetOffset;
        return value < 0 ? value + 1 : value;
    }
}
