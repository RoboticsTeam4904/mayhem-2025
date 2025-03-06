package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

import java.util.HashMap;

/** Sponsored by Claude™ 3.7 Sonnet by Anthropic® */
public class VisionSubsystem extends SubsystemBase {
    public enum TagGroup {
        ANY,
        INTAKE,
        REEF,
        BARGE,
        PROCESSOR
    }

    private static final int TAGS_PER_FIELD_SIDE = 11;
    private static final HashMap<TagGroup, int[]> tagIds = new HashMap<>();

    static {
        // -1 means any tag (needs to be the first item in the array)
        tagIds.put(TagGroup.ANY, new int[] { -1 });
        tagIds.put(TagGroup.INTAKE, new int[] { 1, 2 });
        tagIds.put(TagGroup.REEF, new int[] { 6, 7, 8, 9, 10, 11 });
        tagIds.put(TagGroup.BARGE, new int[] { 4, 5 });
        tagIds.put(TagGroup.PROCESSOR, new int[] { 3 });

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

    private final SwerveDrive swerveDrive;
    private final PhotonCamera photonCamera;

    // pid controllers
    private final PIDController positionController;
    private final PIDController rotationController;

    // target pose relative to tag
    private Transform2d targetPoseRelative;

    // all timeouts in seconds
    private final double CANT_SEE_TIMEOUT = 2; // give up if we cant see the april tag for this many seconds
    private final double TOTAL_TIMEOUT = 5; // always give up after this many seconds
    private double startTime = 0;
    private double lastSeenTagTime = 0;

    // ids for all of the possible tags to target
    private int[] targetTagOptions = null;

    // int if aligning to a certain tag, null if not
    private Integer targetTagId = null;

    // max speeds
    // TODO tune speeds
    private final double MAX_LINEAR_SPEED = 3; // meters per second
    private final double MAX_ROT_SPEED = Math.PI; // radians per second

    // tolerance thresholds for positioning
    private final double POS_TOLERANCE_METERS = 0.05; // 5 cm
    private final double ROT_TOLERANCE_DEG = 2.0; // 2 degrees

    // camera position relative to robot center
    private final Transform2d cameraOffset;

    /**
     * Creates a new VisionSubsystem
     *
     * @param swerveDrive The YAGSL swerve drive subsystem
     * @param photonCamera The PhotonVision camera
     * @param cameraOffset The transform from the camera to the robot center
     */
    public VisionSubsystem(SwerveDrive swerveDrive, PhotonCamera photonCamera, Transform2d cameraOffset) {
        this.swerveDrive = swerveDrive;
        this.photonCamera = photonCamera;
        this.cameraOffset = cameraOffset;

        // initialize pid controllers
        // TODO tune pid values
        positionController = new PIDController(100.0, 0.0, 0.0);
        rotationController = new PIDController(10.0, 0.0, 0.0);

        // make rotation controller continuous
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // set tolerances
        positionController.setTolerance(POS_TOLERANCE_METERS);
        rotationController.setTolerance(Math.toRadians(ROT_TOLERANCE_DEG));

        // target pose relative to tag
        // TODO change depending on selection
        targetPoseRelative = new Transform2d(0, 0.3, Rotation2d.kZero);
    }

    @Override
    public void periodic() {
        if (!this.isPositioning()) return;

        double currentTime = Timer.getFPGATimestamp();

        if (targetTagId == null) {
            // find best tag out of possible and target it
            targetTagId = getBestTargetId(targetTagOptions);
        }

        // get latest result from photonvision
        PhotonTrackedTarget target = targetTagId != null ? getTarget(targetTagId) : null;

        if (target == null) {
            // stop movement since no tags are visible
            // TODO keep moving towards last seen pos
            swerveDrive.drive(new ChassisSpeeds(0, 0, 0));

            // stop positioning if tag has not been seen for a while
            double timeElapsed = currentTime - lastSeenTagTime;
            if (timeElapsed > CANT_SEE_TIMEOUT) {
                stopPositioning("No april tag visible for " + CANT_SEE_TIMEOUT + " seconds");
            }

            return;
        } else {
            lastSeenTagTime = currentTime;
        }

        // get transform from camera to the target
        Transform3d targetOff = target.getBestCameraToTarget();

        // calculate position error relative to desired position
        Transform2d desiredOff = calculatePositionError(
            new Transform2d(
                targetOff.getX(),
                targetOff.getY(),
                targetOff.getRotation().toRotation2d()
            )
        );

        // use pid to calculate needed speeds for x, y, rotation
        double xSpeed = positionController.calculate(0, desiredOff.getX());
        double ySpeed = positionController.calculate(0, desiredOff.getY());
        double rotSpeed = rotationController.calculate(0, desiredOff.getRotation().getRadians());

        xSpeed = Util.clamp(xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        ySpeed = Util.clamp(ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        rotSpeed = Util.clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

        // convert to robot relative speeds
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            ySpeed,
            -xSpeed,
            rotSpeed,
            swerveDrive.getOdometryHeading()
        );

        // command swerve drive
        swerveDrive.drive(fieldRelativeSpeeds);

        // log positioning data
        System.out.printf(
            "Positioning to april tag %s... %s X, %s Y, %sdeg%n",
            targetTagId,
            Math.round(desiredOff.getX() * 1000) / 1000,
            Math.round(desiredOff.getY() * 1000) / 1000,
            desiredOff.getRotation().getDegrees()
        );

        // check if reached target position
        boolean atPosition =
            Math.hypot(desiredOff.getX(), desiredOff.getY()) < POS_TOLERANCE_METERS &&
            Math.abs(desiredOff.getRotation().getDegrees()) < ROT_TOLERANCE_DEG;

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
    private Integer getBestTargetId(int[] tagIds) {
        PhotonPipelineResult result = photonCamera.getLatestResult();

        if (!result.hasTargets()) return null;

        // -1 represents any, so return any best result
        if (tagIds[0] == -1) {
            return result.getBestTarget().fiducialId;
        }

        // find best tag in the possible tag options
        for (var target : result.getTargets()) {
            for (int tagId : tagIds) {
                if (tagId == target.fiducialId) {
                    return tagId;
                }
            }
        }

        return null;
    }

    /**
     * Get a PhotonVision target for an April Tag matching a certain ID
     *
     * @param tagId The ID of the tag to look for
     * @return A {@link PhotonTrackedTarget} or {@code null} if no April Tag was found
     */
    PhotonTrackedTarget getTarget(int tagId) {
        PhotonPipelineResult result = photonCamera.getLatestResult();

        for (var target : result.getTargets()) {
            if (tagId == target.fiducialId) {
                return target;
            }
        }

        return null;
    }

    private void startPositioning(int[] targetTagIds, Transform2d targetPoseRelative) {
        this.targetTagOptions = targetTagIds;
        this.targetPoseRelative = targetPoseRelative;

        startTime = lastSeenTagTime = Timer.getFPGATimestamp();

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
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0));

        System.out.println("Positioning ended" + (status != null ? " - " + status : ""));
    }

    /**
     * Calculate the position error between current position and desired position
     *
     * @param targetOff The transform from camera to the tag
     * @return The transform from current position to desired position
     */
    private Transform2d calculatePositionError(Transform2d targetOff) {
        // calculate transform from robot to the tag
        Transform2d robotToTarget = new Transform2d(
            targetOff.getTranslation().plus(cameraOffset.getTranslation().rotateBy(targetOff.getRotation())),
            targetOff.getRotation().plus(cameraOffset.getRotation())
        );

        // calculate desired robot to target transform
        Transform2d desiredRobotToTarget = targetPoseRelative;

        // calculate difference between current and desired
        Translation2d translationError = desiredRobotToTarget.getTranslation().minus(robotToTarget.getTranslation());
        Rotation2d rotationError = desiredRobotToTarget.getRotation().minus(robotToTarget.getRotation());

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
     * Apply a deadband to a value
     *
     * @param value The input value
     * @param deadband The deadband range
     * @return The value with deadband applied
     */
    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    // TODO tune
    public final Transform2d CAMERA_OFFSET = new Transform2d(
        0, 0, Rotation2d.kPi
    );

    /**
     * Start aligning to an April Tag that matches the ID given
     *
     * @param targetTagId The ID of the April Tag to align to
     */
    public Command c_align(int targetTagId) {
        return c_align(new int[] { targetTagId });
    }

    /**
     * Start aligning to an April Tag that matches the {@link TagGroup} given
     *
     * @param targetTagGroup A group of april tags to align to, e.g. {@code TagGroup.REEF}.
     *                       The robot will align to whichever one PhotonVision considers the best
     */
    public Command c_align(TagGroup targetTagGroup) {
        return c_align(tagIds.get(targetTagGroup));
    }

    /**
     * Start aligning to an April Tag that matches one of the IDs given
     *
     * @param targetTagIds A list of april tag IDs to align to.
     *                     The robot will align to whichever one PhotonVision considers the best
     */
    public Command c_align(int[] targetTagIds) {
        var command = new SequentialCommandGroup(
            this.runOnce(() -> startPositioning(targetTagIds, CAMERA_OFFSET)),
            new WaitWhile(this::isPositioning)
        ) {
            @Override
            public void cancel() {
                super.cancel();
                stopPositioning();
            }
        };
        command.addRequirements(RobotMap.Component.chassis);
        return command;
    }

    public Command c_stop() {
        return new InstantCommand(this::stopPositioning);
    }
}
