package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

public class LightSubsystem extends SubsystemBase {

    public static class Color {

        public static final int[] SUCCESS  = { 0, 200, 50 };
        public static final int[] FAIL     = { 255, 50, 0 };

        public static final int[] VISION   = { 127, 0, 255 };
        public static final int[] ELEVATOR = { 255, 240, 0 };

        public static final int[] ENABLED  = { 255, 100, 0 };
        public static final int[] DISABLED = { 200, 220, 255 };

    }

    public double visionProgress = -1;
    public double elevatorProgress = -1;

    private float[] flashColor = new float[4];
    private float flashStrength = 0;

    double lastUpdateTime;

    public static class BufferViewData {

        public final AddressableLED led;
        public final int length;
        public final AddressableLEDBufferView view;

        public BufferViewData(AddressableLED led, int length, int start, int end) {
            this.led = led;
            this.length = end - start;

            if (!bufferMap.containsKey(led)) {
                bufferMap.put(led, new AddressableLEDBuffer(length));
                led.setLength(length);
                led.start();
            }
            view = new AddressableLEDBufferView(getBuffer(), start, end);
        }

        public static final Map<AddressableLED, AddressableLEDBuffer> bufferMap = new HashMap<>();

        public AddressableLEDBuffer getBuffer() {
            return bufferMap.get(led);
        }

    }

    final BufferViewData[] views;

    public LightSubsystem(BufferViewData... views) {
        this.views = views;

        lastUpdateTime = Timer.getFPGATimestamp();
    }

    private float[] alphaBlend(float[] a, float[] b) {
        return alphaBlend(a, b, 1);
    }

    private float[] alphaBlend(float[] a, float[] b, float mix) {
        float alpha = b[3] * mix;

        // this math might not be entirely correct if the alpha component of a is < 1, but it's close enough
        return new float[] {
            a[0] * (1 - alpha) + b[0] * alpha,
            a[1] * (1 - alpha) + b[1] * alpha,
            a[2] * (1 - alpha) + b[2] * alpha,
            1 - (1 - a[3]) * (1 - b[3])
        };
    }

    private float[][] progressBar(float progress, int[] color, int length) {
        float[][] colors = new float[length][4];

        for (int i = 0; i < length; i++) {
            float strength = Math.min(1, progress * length - i);
            if (strength > 0) {
                colors[i] = new float[] {
                    color[0] / 255f,
                    color[1] / 255f,
                    color[2] / 255f,
                    color[3] * strength
                };
            }
        }

        return colors;
    }

    /**
     * Flash an RGB color for about a second. Color is an array of 3 (RGB) or 4 (RGBA) ints from 0-255.
     */
    public void flashColor(int[] color) {
        if (color.length == 3) {
            flashColor(color[0], color[1], color[2]);
        } else if (color.length == 4) {
            flashColor(color[0], color[1], color[2], color[3]);
        } else {
            System.err.println("LightSubsystem.flashColor(int[] color) must take an array of 3 or 4 ints for RGB or RGBA");
            flashColor(0, 0, 0);
        }
    }

    /**
     * Flash an RGB color for about a second. Colors are from 0-255.
     */
    public void flashColor(int r, int g, int b) {
        flashColor(r, g, b, 255);
    }

    /**
     * Flash an RGB color for about a second. Colors and alpha are from 0-255.
     */
    public void flashColor(int r, int g, int b, int a) {
        flashColor = new float[] { r / 255f, g / 255f, b / 255f, a / 255f };
        flashStrength = 1;
    }

    @Override
    public void periodic() {
        double time = Timer.getFPGATimestamp();
        double deltaTime = time - lastUpdateTime;
        lastUpdateTime = time;

        for (var view : views) {
            float[][] colors;

            if (visionProgress != -1) {
                colors = progressBar((float) visionProgress, Color.VISION, view.length);
            } else if (elevatorProgress != -1) {
                colors = progressBar((float) elevatorProgress, Color.ELEVATOR, view.length);
            } else {
                colors = new float[view.length][4];
            }

            if (flashStrength > 0) {
                for (int i = 0; i < view.length; i++) {
                    colors[i] = alphaBlend(colors[i], flashColor, flashStrength);
                }

                flashStrength -= (float) deltaTime;
            }

            setViewColors(view.view, colors);
        }

        for (var led : BufferViewData.bufferMap.keySet()) {
            led.setData(BufferViewData.bufferMap.get(led));
        }
    }

    private void setViewColors(AddressableLEDBufferView view, float[][] colors) {
        for (int i = 0; i < colors.length; i++) {
            float[] color = colors[i];
            float scale = color[3] * 255;

            view.setRGB(
                i,
                (int) (color[0] * scale),
                (int) (color[1] * scale),
                (int) (color[2] * scale)
            );
        }
    }
}
