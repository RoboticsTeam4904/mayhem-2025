package org.usfirst.frc4904.standard.custom;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CustomEncoder {
    public final DutyCycleEncoder encoder;

    private double resetOffset = 0;

    public CustomEncoder(DutyCycleEncoder encoder) {
        this.encoder = encoder;
        reset();
    }

    public void reset() {
        resetOffset = encoder.get();
    }

    int revolutions = 0;
    Double lastReading = null;

    public double get() {
        double reading = encoder.get();

        if (lastReading == null) {
            lastReading = reading;
            return reading;
        }

        if (Math.abs(reading - lastReading) >= 0.5) {
            if (reading < lastReading) {
                revolutions++;
            } else {
                revolutions--;
            }
        }

        lastReading = reading;

        return reading + revolutions - resetOffset;
    }
}
