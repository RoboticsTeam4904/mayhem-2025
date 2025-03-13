package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CustomEncoder {
    public final DutyCycleEncoder encoder;

    public CustomEncoder(DutyCycleEncoder encoder) {
        this.encoder = encoder;
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

        return reading + revolutions;
    }
}
