package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.Perlin2D;
import org.usfirst.frc4904.standard.Util;

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
        public final float[][] colorArray;
        public final AddressableLEDBufferView view;

        public BufferViewData(AddressableLED led, int length, int start, int end) {
            this.led = led;
            this.length = end - start;
            this.colorArray = new float[length][4];

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

    private void alphaBlend(float[][] colors, float[] c2, float mix) {
        float alpha = c2[3] * mix;

        for (int i = 0; i < colors.length; i++) {
            // this math might not be entirely correct if the alpha component of a is < 1, but it's close enough
            colors[i][0] = colors[i][0] * (1 - alpha) + c2[0] * alpha;
            colors[i][1] = colors[i][1] * (1 - alpha) + c2[1] * alpha;
            colors[i][2] = colors[i][2] * (1 - alpha) + c2[2] * alpha;
            colors[i][3] = 1 - (1 - colors[i][3]) * (1 - c2[3]);
        }
    }

    private void progressBar(float[][] colors, float progress, int[] color) {
        int length = colors.length;

        for (int i = 0; i < length; i++) {
            float strength = Util.clamp(progress * length - i, 0, 1);
            if (strength > 0) {
                colors[i][0] = color[0] / 255f;
                colors[i][1] = color[1] / 255f;
                colors[i][2] = color[2] / 255f;
            }
            colors[i][3] = color[3] * strength;
        }
    }

    private final Perlin2D fireNoise = new Perlin2D(12345678987654321L);

    private void fire(float[][] colors) {
        float time = (float) lastUpdateTime;

        for (int i = 0; i < colors.length; i++) {
            float noise = fireNoise.noise(time, (float) Math.pow((float) i / colors.length, 2) * 200 + time * 2);
            float strength = (float) Math.pow(noise, 2.5) + i * 1.2f - 0.5f;

            colors[i][0] = 1;
            colors[i][1] = Util.clamp(strength * 2 - 0.5f, 0, 1);
            colors[i][2] = Util.clamp(strength * 4 - 3, 0, 1);
            colors[i][3] = Util.clamp(strength * 4, 0, 1);
        }
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
            float[][] colors = view.colorArray;

            if (visionProgress != -1) {
                progressBar(colors, (float) visionProgress, Color.VISION);
            } else if (elevatorProgress != -1) {
                progressBar(colors, (float) elevatorProgress, Color.ELEVATOR);
            } else {
                fire(colors);
            }

            if (flashStrength > 0) {
                alphaBlend(colors, flashColor, flashStrength);
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
