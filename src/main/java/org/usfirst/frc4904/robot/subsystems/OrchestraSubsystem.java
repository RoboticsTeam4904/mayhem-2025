package org.usfirst.frc4904.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

/** Orchestra™ */
public class OrchestraSubsystem extends SubsystemBase {

    private static final HashMap<String, OrchestraSubsystem> songs = new HashMap<>();

    /**
     * Loads a song to be played later with {@link OrchestraSubsystem#playSong(String)}.
     * <p>
     * Files for the song should be located at {@code deploy/chirp/<name>_<track>.chrp}
     *
     * @param name The name of the song to load.
     * @param tracks The number of tracks the song has.
     * @param motors The motors that will be used to play the song. There should be at least as many motors as there are tracks.
     */
    public static void loadSong(String name, int tracks, CANTalonFX... motors) {
        songs.put(name, new OrchestraSubsystem(name, tracks, motors));
    }

    /**
     * Play a song that was already loaded with {@link OrchestraSubsystem#loadSong(String, int, CANTalonFX...)}.
     * @param name The name of the song to play.
     * @return Whether the song was successfully played.
     */
    public static boolean playSong(String name) {
        OrchestraSubsystem orchestra = songs.get(name);

        if (orchestra == null) {
            System.out.println("Song '" + name + "' does not exist");
            return false;
        }

        return orchestra.play();
    }

    public static String PATH = "/home/lvuser/deploy/chirp/";

    private final List<Orchestra> orchestras = new ArrayList<>();
    public final String songName;

    private OrchestraSubsystem(String name, int tracks, CANTalonFX... motors) {
        songName = name;

        if (motors.length < tracks) {
            System.out.printf(
                "Not enough motors for song '%s' (Got: %d, Recommended: %d).%n",
                name,
                motors.length,
                tracks
            );
        }

        for (int track = 0; track < tracks; track++) {
            var orchestra = new Orchestra();

            String path = PATH + name + "_" + track + ".chrp";
            if (!orchestra.loadMusic(path).isOK()) {
                System.out.printf(
                    "Failed to load song '%s', track %d. Make sure the file '%s' exists.%n",
                    path,
                    track,
                    path
                );
            }

            for (int i = track; i < motors.length; i += tracks) {
                orchestra.addInstrument(motors[i], 0);
            }

            orchestras.add(orchestra);
        }
    }

    public boolean play() {
        boolean success = true;

        for (int i = 0; i < orchestras.size(); i++) {
            if (!orchestras.get(i).play().isOK()) {
                System.out.println("Song '" + songName + "', track " + i + " failed to play");
                success = false;
            }
        }

        return success;
    }
}
