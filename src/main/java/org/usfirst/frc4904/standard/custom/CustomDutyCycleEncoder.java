package org.usfirst.frc4904.standard.custom;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CustomDutyCycleEncoder extends DutyCycleEncoder {
    private int channel;
    private double zeroOffset;
    private double resetOffset = 0;

    public CustomDutyCycleEncoder(int channel) {
        super(channel, 1, entry(channel).get());

        this.channel = channel;
        this.zeroOffset = entry(channel).get();
    }

    private static DoubleEntry entry(int channel) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        DoubleTopic topic = inst.getDoubleTopic("/zeros/" + channel);
        topic.setPersistent(true);

        return topic.getEntry(0.0);
    }

    public void reset() {
        resetOffset = super.get();
        entry(channel).set(zeroOffset + resetOffset);
    }

    @Override
    public double get() {
        double value = super.get() - resetOffset;
        return value < 0 ? value + 1 : value;
    }
}
