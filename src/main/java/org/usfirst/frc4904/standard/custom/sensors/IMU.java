package org.usfirst.frc4904.standard.custom.sensors;

public interface IMU {
    /**
     * Resets the IMU.
     */
    void reset();

    /**
     * @return Rate of rotation about yaw axis
     */
    double getRate();

    /**
     * @return Current yaw value
     */
    float getYaw();

    /**
     * @return Current pitch value
     */
    float getPitch();

    /**
     * @return Current roll value
     */
    float getRoll();
}
