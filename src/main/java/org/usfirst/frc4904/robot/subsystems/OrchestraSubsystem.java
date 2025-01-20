package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

/** Orchestra™ */
public class OrchestraSubsystem extends SubsystemBase {

    private static final HashMap<String, OrchestraSubsystem> orchestras = new HashMap<>();

    /**
     * Register an instance of <code>OrchestraSubsystem</code> to be played later.
     * @param name The name of the song.
     * @param orchestra The instance of <code>OrchestraSubsystem</code>.
     */
    public static void addSong(String name, OrchestraSubsystem orchestra) {
        orchestras.put(name, orchestra);
    }

    /**
     * Play a song that was already created with <code>OrchestraSubsystem.addSong()</code>.
     * @param name The name of the song to play.
     * @return A {@link Command} that plays the song (or a {@link Noop} command if the song does not exist).
     */
    public static Command c_playSong(String name) {
        OrchestraSubsystem orchestra = orchestras.get(name);

        System.out.println("tried play" + name);

        if (orchestra == null) {
            System.out.println("Song '" + name + "' does not exist");
            return new Noop();
        }

        System.out.println("going through with play" + name);

        return orchestra.c_play();
    }

    Orchestra orchestra = new Orchestra();

    /**
     * Create an instance of <code>OrchestraSubsystem</code> to be played later. Use with <code>OrchestraSubsystem.addSong()</code>.
     *
     * @param file The file path of the .chrp file for the song. Needs to be in robot/chirp.
     * @param tracks The amount of tracks in the song.
     * @param motors The motors that will be used to play the song. One motor will be assigned to each track, and once
     *               all tracks are filled, it will wrap around and start assigning multiple motors to tracks.
     *               There should be at least as many motors as there are tracks in the song.
     */
    public OrchestraSubsystem(String file, int tracks, CANTalonFX... motors) {
        System.out.println("Initializing Orchestra with file: " + file);
        if (motors.length < tracks) {
            DriverStation.reportWarning(
                String.format(
                    "Not enough motors for song '%s' (Got: %d, Recommended: %d)",
                    file,
                    motors.length,
                    tracks
                ),
                false
            );
        }

        System.out.println("Adding " + motors.length + " instruments...");
        for (int i = 0; i < motors.length; i++) {
            orchestra.addInstrument(motors[i], i % tracks);
        }

        boolean loadSuccess = orchestra.loadMusic(file);
        if (!loadSuccess) {
            DriverStation.reportError(
                "Failed to load music file: " +
                file +
                ". Verify the file exists and path is correct.",
                false
            );
        } else {
            System.out.println("Successfully loaded music file: " + file);
        }
    }

    public Command c_play() {
        System.out.println("Attempting to play music...");
        String musicFile = "deploy/chirp/delfino.chrp";
        var loadStatus = orchestra.loadMusic(musicFile);
        if (!loadStatus.isOK()) {
            DriverStation.reportError(
                "Failed to reload music file: " +
                musicFile +
                " during play attempt. Status: " +
                loadStatus.toString(),
                false
            );
            return new Noop();
        }
        System.out.println("Music loaded successfully, starting playback...");
        var playStatus = orchestra.play();
        System.out.println("Play command status: " + playStatus.toString());
        return this.run(() -> orchestra.play());
    }
}
