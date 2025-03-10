package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.Util;
import org.usfirst.frc4904.standard.commands.WaitWhile;
import swervelib.SwerveDrive;

import java.util.*;

/** Sponsored by Claude™ 3.7 Sonnet by Anthropic® */
public class VisionSubsystem extends SubsystemBase {
    public enum TagGroup {
        ANY,
        INTAKE,
        REEF
    }

    private static final int TAGS_PER_FIELD_SIDE = 11;
    private static final HashMap<TagGroup, int[]> tagIds = new HashMap<>();

    static {
        // -1 means any tag (needs to be the first item in the array)
        tagIds.put(TagGroup.ANY, new int[] { -1 });
        tagIds.put(TagGroup.INTAKE, new int[] { 1, 2 });
        tagIds.put(TagGroup.REEF, new int[] { 6, 7, 8, 9, 10, 11 });

        // move april tags to other side of board if on the blue alliance
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue) {
            for (var ids : tagIds.values()) {
                if (ids[0] == -1) continue; // skip ANY

                for (int i = 0; i < ids.length; i++) {
                    ids[i] += TAGS_PER_FIELD_SIDE;
                }
            }
        }
    }

    private record CameraTag(PhotonTrackedTarget tag, int cameraIndex) {}

    private final SwerveDrive swerveDrive;
    private final PhotonCamera[] photonCameras;

    // pid controllers
    private final PIDController positionController;
    private final PIDController rotationController;

    // target pose relative to tag
    private Transform2d offset;

    // all timeouts in seconds
    private final double CANT_SEE_TIMEOUT = 1; // give up if we cant see the april tag for this many seconds
    private final double TOTAL_TIMEOUT = 5; // always give up after this many seconds
    private double startTime = 0;
    private double lastSeenTagTime = 0;

    // ids for all of the possible tags to target
    private int[] targetTagOptions = null;

    // int if aligning to a certain tag, null if not
    private Integer targetTagId = null;
    private Transform2d desiredOffset = null;

    // used to estimate position when we can't see the tag
    private double lastTime;
    private ChassisSpeeds lastSpeed = null;

    // max speeds
    // TODO tune speeds
    private final double MAX_LINEAR_SPEED = 2; // meters per second
    private final double MAX_ROT_SPEED = Math.PI; // radians per second

    // tolerance thresholds for positioning
    private final double POS_TOLERANCE_METERS = 0.02;
    private final double ROT_TOLERANCE_DEG = 1.0;

    // camera positions relative to robot center
    private final Transform2d[] cameraOffsets;

    /**
     * Creates a new VisionSubsystem
     *
     * @param swerveDrive The YAGSL swerve drive subsystem
     * @param photonCameras The PhotonVision cameras
     * @param cameraOffsets The transforms from the camera to the robot center
     */
    public VisionSubsystem(SwerveDrive swerveDrive, PhotonCamera[] photonCameras, Transform2d[] cameraOffsets) {
        this.swerveDrive = swerveDrive;
        this.photonCameras = photonCameras;
        this.cameraOffsets = cameraOffsets;

        // initialize pid controllers
        // TODO tune pid values
        positionController = new PIDController(5.0, 0.0, 0.0);
        rotationController = new PIDController(7.5, 0.0, 0.0);

        // make rotation controller continuous
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // set tolerances
        positionController.setTolerance(POS_TOLERANCE_METERS);
        rotationController.setTolerance(Math.toRadians(ROT_TOLERANCE_DEG));
    }

    @Override
    public void periodic() {
        if (!this.isPositioning()) return;

        double currentTime = Timer.getFPGATimestamp();

        CameraTag target;

        if (targetTagId == null) {
            // find best tag out of possible tags and target it
            target = getBestTargetId(targetTagOptions);
            targetTagId = target != null ? target.tag.fiducialId : null;
        } else {
            target = getTarget(targetTagId);
        }

        if (target == null) {
            // stop positioning if tag has not been seen for a while
            double timeElapsed = currentTime - lastSeenTagTime;
            if (timeElapsed > CANT_SEE_TIMEOUT) {
                stopPositioning("No april tag visible for " + CANT_SEE_TIMEOUT + " seconds");
                return;
            }

            if (lastSpeed == null || desiredOffset == null) return;

            double deltaTime = currentTime - lastTime;
            desiredOffset = desiredOffset.plus(
                new Transform2d(
                    -lastSpeed.vxMetersPerSecond * deltaTime,
                    -lastSpeed.vyMetersPerSecond * deltaTime,
                    new Rotation2d(-lastSpeed.omegaRadiansPerSecond * deltaTime)
                )
            );
        } else {
            lastSeenTagTime = currentTime;

            // calculate position error relative to desired position
            desiredOffset = calculatePositionError(target);
        }

        // use pid to calculate needed speeds for x, y, rotation
        double xSpeed = positionController.calculate(0, desiredOffset.getX());
        double ySpeed = positionController.calculate(0, desiredOffset.getY());
        double rotSpeed = rotationController.calculate(0, desiredOffset.getRotation().getRadians());

        xSpeed = Util.clamp(xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        ySpeed = Util.clamp(ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        rotSpeed = Util.clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

        // convert to robot relative speeds
        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xSpeed,
            ySpeed,
            rotSpeed,
            Rotation2d.kZero
        );

        // command swerve drive
        swerveDrive.drive(relativeSpeeds);

        lastTime = currentTime;
        lastSpeed = relativeSpeeds;

        // log positioning data
        System.out.printf(
            "Positioning to tag %s. X: %.4f, Y: %.4f, Rot: %.2fdeg%n",
            targetTagId,
            desiredOffset.getX(),
            desiredOffset.getY(),
            desiredOffset.getRotation().getDegrees()
        );

        // check if reached target position
        boolean atPosition =
            Math.hypot(desiredOffset.getX(), desiredOffset.getY()) < POS_TOLERANCE_METERS &&
            Math.abs(desiredOffset.getRotation().getDegrees()) < ROT_TOLERANCE_DEG;

        if (atPosition) {
            stopPositioning("Success");
        } else {
            // give up if too much time has passed
            double timeElapsed = currentTime - startTime;
            if (timeElapsed > TOTAL_TIMEOUT) {
                stopPositioning("Gave up after " + TOTAL_TIMEOUT + " seconds");
            }
        }
    }

    /**
     * Find the best April Tag to target given a list of April Tag IDs
     *
     * @param tagIds The IDs of the April Tags to target
     * @return The ID of the best April Tag, or null if none were found
     */
    private CameraTag getBestTargetId(int[] tagIds) {
        List<CameraTag> results = getResults();

        if (results.isEmpty()) return null;

        // -1 represents any, so return any best result
        if (tagIds[0] == -1) {
            return results.get(0);
        }

        // find best tag in the possible tag options
        for (var target : results) {
            for (int tagId : tagIds) {
                if (tagId == target.tag.fiducialId) {
                    return target;
                }
            }
        }

        return null;
    }

    /**
     * Get a PhotonVision target for an April Tag matching a certain ID
     *
     * @param tagId The ID of the tag to look for
     * @return A {@link CameraTag} or {@code null} if no April Tag was found
     */
    CameraTag getTarget(int tagId) {
        for (var target : getResults()) {
            if (tagId == target.tag.fiducialId) {
                return target;
            }
        }

        return null;
    }

    /**
     * @return A list of possible targets
     */
    List<CameraTag> getResults() {
        List<CameraTag> results = new ArrayList<>();

        for (int i = 0; i < photonCameras.length; i++) {
            List<PhotonPipelineResult> unreadResults = photonCameras[i].getAllUnreadResults();
            if (unreadResults.isEmpty()) continue;

            double lastCaptureTime = 0;
            for (var result : unreadResults) {
                lastCaptureTime = Math.max(lastCaptureTime, result.getTimestampSeconds());
            }

            for (var result : unreadResults) {
                // discard results that are more than 0.1 seconds older than the latest result
                if (lastCaptureTime - result.getTimestampSeconds() > 0.1) continue;

                for (var target : result.getTargets()) {
                    results.add(new CameraTag(target, i));
                }
            }
        }

        results.sort(Comparator.comparingDouble(result -> {
            Transform3d transform = result.tag.getBestCameraToTarget();
            return Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        }));

        return results;
    }

    private void startPositioning(int[] targetTagIds, Transform2d offset) {
        System.out.println("OFFSET 4: " + offset.getX() + " " + offset.getY()); // TODO remove
        this.targetTagOptions = targetTagIds;
        this.offset = offset;

        startTime = lastSeenTagTime = lastTime = Timer.getFPGATimestamp();

        // reset pid controllers
        positionController.reset();
        rotationController.reset();

        System.out.println("Positioning started");
    }

    /**
     * Stop the positioning process
     */
    public void stopPositioning() {
        stopPositioning(null);
    }

    /**
     * Stop the positioning process
     *
     * @param status The status to log after positioning, e.g. "Success"
     */
    public void stopPositioning(String status) {
        targetTagOptions = null;
        targetTagId = null;
        desiredOffset = null;
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0));

        System.out.println("Positioning ended" + (status != null ? " - " + status : ""));
    }

    /**
     * Calculate the position error between current position and desired position
     *
     * @param target The target to calculate alignment to
     * @return The transform from current position to desired position
     */
    private Transform2d calculatePositionError(CameraTag target) {
        Transform3d rawOffset = target.tag.getBestCameraToTarget();

        Transform2d targetOffset = new Transform2d(
            rawOffset.getX(),
            rawOffset.getY(),
            rawOffset.getRotation().toRotation2d()
        );

        Transform2d cameraOffset = cameraOffsets[target.cameraIndex];

        // calculate transform from robot to the tag
        Transform2d robotToTarget = new Transform2d(
            targetOffset.getTranslation().plus(cameraOffset.getTranslation().rotateBy(targetOffset.getRotation())),
            targetOffset.getRotation().plus(cameraOffset.getRotation())
        );

        System.out.println("OFFSET 5: " + offset.getX() + " " + offset.getY()); // TODO remove

        // calculate difference between current and desired
        Translation2d translationError = offset.getTranslation().minus(robotToTarget.getTranslation());
        Rotation2d rotationError = offset.getRotation().minus(robotToTarget.getRotation());

        return new Transform2d(translationError, rotationError);
    }

    /**
     * Check if the subsystem is currently attempting to position
     *
     * @return true if positioning is in progress
     */
    public boolean isPositioning() {
        return targetTagOptions != null;
    }

    /**
     * Start aligning to an April Tag that matches the ID given
     *
     * @param targetTagId The ID of the April Tag to align to
     */
    public Command c_align(int targetTagId, Transform2d offset) {
        return c_align(new int[] { targetTagId }, offset);
    }

    /**
     * Start aligning to an April Tag that matches the {@link TagGroup} given
     *
     * @param targetTagGroup A group of april tags to align to, e.g. {@code TagGroup.REEF}.
     *                       The robot will align to whichever one PhotonVision considers the best
     */
    public Command c_align(TagGroup targetTagGroup, Transform2d offset) {
        System.out.println("OFFSET 1: " + (offset == null ? "null" : offset.getX() + " " + offset.getY())); // TODO remove
        return c_align(tagIds.get(targetTagGroup), offset);
    }

    /**
     * Start aligning to an April Tag that matches one of the IDs given
     *
     * @param targetTagIds A list of april tag IDs to align to.
     *                     The robot will align to whichever one PhotonVision considers the best
     */
    public Command c_align(int[] targetTagIds, Transform2d offset) {
        System.out.println("OFFSET 2: " + (offset == null ? "null" : offset.getX() + " " + offset.getY())); // TODO remove
        final Transform2d finalOffset = offset != null ? offset : Transform2d.kZero;
        System.out.println("OFFSET 3: " + finalOffset.getX() + " " + finalOffset.getY()); // TODO remove

        var command = new SequentialCommandGroup(
            this.runOnce(() -> startPositioning(targetTagIds, finalOffset)),
            new WaitWhile(this::isPositioning)
        ) {
            @Override
            public void cancel() {
                super.cancel();
                stopPositioning("Command canceled");
            }
        };
        command.addRequirements(RobotMap.Component.chassis);
        return command;
    }

    public Command c_stop() {
        return new InstantCommand(() -> this.stopPositioning("Stop command"));
    }
}
