package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        // i know we don't need to align to these but i think it's funny
        BARGE,
        COMPACTOR
    }

    private static final int TAGS_PER_FIELD_SIDE = 11;
    private static final HashMap<TagGroup, int[]> tagIds = new HashMap<>();

    static {
        // -1 means any tag. only works if it is the first item in the array b/c there should never be any other items anyway
        tagIds.put(TagGroup.ANY, new int[] { -1 });
        tagIds.put(TagGroup.INTAKE, new int[] { 1, 2 });
        tagIds.put(TagGroup.REEF, new int[] { 6, 7, 8, 9, 10, 11 });
        tagIds.put(TagGroup.BARGE, new int[] { 4, 5 });
        tagIds.put(TagGroup.COMPACTOR, new int[] { 3 });

        // move april tags to other side of board if we are on the blue alliance
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue) {
            for (var ids : tagIds.values()) {
                for (int i = 0; i < ids.length; i++) {
                    ids[i] += TAGS_PER_FIELD_SIDE;
                }
            }
        }
    }

    private final SwerveDrive swerveDrive;
    private final PhotonCamera photonCamera;

    // PID controllers for X, Y positions and rotation
    private final PIDController positionController;
    private final PIDController rotationController;

    // Target pose relative to the AprilTag
    private Transform2d targetPoseRelative;

    // Timeout for positioning
    private final double TIMEOUT_SECONDS = 3.0; // give up if we lose sight of the april tag for this long
    private double startTime = 0;
    // ids for all of the tags we can align to (e.g. the 6 ids for the different sides of the reef)
    // only used in the brief time when alignment has been started but no tags are visible to find the best tag
    private int[] targetTagOptions = null;
    // int if we are aligning, null if we are not
    private Integer targetTagId = null;

    // Tolerance thresholds for positioning
    private final double POS_TOLERANCE_METERS = 0.05; // 5cm
    private final double ROT_TOLERANCE_DEG = 2.0; // 2 degrees

    // Camera mounting position relative to robot center
    private final Transform2d cameraToRobot;

    /**
     * Creates a new AprilTagPositioningSubsystem
     *
     * @param swerveDrive The YAGSL swerve drive subsystem
     * @param photonCamera The PhotonVision camera
     * @param cameraToRobot The transform from the camera to the robot center
     */
    public VisionSubsystem(SwerveDrive swerveDrive, PhotonCamera photonCamera, Transform2d cameraToRobot) {
        this.swerveDrive = swerveDrive;
        this.photonCamera = photonCamera;
        this.cameraToRobot = cameraToRobot;

        // Initialize PID controllers
        // TODO tune maybe
        positionController = new PIDController(1.0, 0.0, 0.0);
        rotationController = new PIDController(1.0, 0.0, 0.0);

        // Make rotation controller continuous
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Set tolerances
        positionController.setTolerance(POS_TOLERANCE_METERS);
        rotationController.setTolerance(Math.toRadians(ROT_TOLERANCE_DEG));

        // Default target is 0,0,0 relative to AprilTag
        targetPoseRelative = new Transform2d(0, 0, new Rotation2d(0));
    }

    @Override
    public void periodic() {
        if (!this.isPositioning()) return;

        // if we haven't seen any tags that we want to align to yet, try to find one again
        // once we find a tag, make sure to only align to that one tag and not switch to a different one
        if (targetTagId == null) {
            targetTagId = getBestTargetId(targetTagOptions);
        }

        // Get the latest result from PhotonVision
        PhotonTrackedTarget target = targetTagId != null ? getTarget(targetTagId) : null;

        if (target == null) {
            // No AprilTags visible, stop movement
            swerveDrive.drive(new ChassisSpeeds(0, 0, 0));

            // If we've lost sight of the AprilTag for too long, stop positioning
            if (Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS) {
                stopPositioning();
                SmartDashboard.putString("Positioning Status", "Failed - No AprilTag visible");
            }

            return;
        }

        // Get the transform from the camera to the target
        Transform3d cameraToTarget = target.getBestCameraToTarget();

        // Calculate the current position error relative to our desired position
        Transform2d currentToDesired = calculatePositionError(
            new Transform2d(
                cameraToTarget.getX(),
                cameraToTarget.getY(),
                cameraToTarget.getRotation().toRotation2d()
            )
        );

        // Use PID to calculate the needed speeds for X, Y, and rotation
        double xSpeed = positionController.calculate(0, currentToDesired.getX());
        double ySpeed = positionController.calculate(0, currentToDesired.getY());
        double rotSpeed = rotationController.calculate(0, currentToDesired.getRotation().getRadians());

        // Apply deadbands and clamp values for safety
        xSpeed = applyDeadband(xSpeed, 0.05);
        ySpeed = applyDeadband(ySpeed, 0.05);
        rotSpeed = applyDeadband(rotSpeed, 0.05);

        // Max speeds (adjust based on your robot)
        double maxLinearSpeed = 2.0;  // meters per second
        double maxRotSpeed = Math.PI; // radians per second

        xSpeed = Util.clamp(xSpeed, -maxLinearSpeed, maxLinearSpeed);
        ySpeed = Util.clamp(ySpeed, -maxLinearSpeed, maxLinearSpeed);
        rotSpeed = Util.clamp(rotSpeed, -maxRotSpeed, maxRotSpeed);

        // Convert to field-relative speeds
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rotSpeed,
            swerveDrive.getOdometryHeading()
        );

        // Command the swerve drive
        swerveDrive.drive(fieldRelativeSpeeds);

        // Update SmartDashboard with positioning data
        SmartDashboard.putNumber("Position Error X", currentToDesired.getX());
        SmartDashboard.putNumber("Position Error Y", currentToDesired.getY());
        SmartDashboard.putNumber("Rotation Error (deg)", Math.toDegrees(currentToDesired.getRotation().getRadians()));

        // Check if we've reached the target position
        boolean atPosition =
            Math.abs(currentToDesired.getX()) < POS_TOLERANCE_METERS &&
            Math.abs(currentToDesired.getY()) < POS_TOLERANCE_METERS &&
            Math.abs(currentToDesired.getRotation().getDegrees()) < ROT_TOLERANCE_DEG;

        // Check if we've timed out
        boolean timedOut = Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS;

        if (atPosition || timedOut) {
            stopPositioning();
        }
    }

    /**
     * Find the best april tag to target given a list of april tag IDs
     *
     * @param tagIds The IDs of the april tags we want to target
     * @return The ID of the best april tag, or null if none were found
     */
    private Integer getBestTargetId(int[] tagIds) {
        PhotonPipelineResult result = photonCamera.getLatestResult();

        if (!result.hasTargets()) return null;

        // -1 represents any, so just return the best result
        if (tagIds[0] == -1) {
            return result.getTargets().getFirst().fiducialId;
        }

        // find the best tag that matches one of the IDs we are looking for
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
     * Get a PhotonVision target for an april tag matching a certain ID
     *
     * @param tagId The ID of the tag to look for
     * @return A {@link PhotonTrackedTarget} or {@code null} if no april tag was found
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
        this.startTime = Timer.getFPGATimestamp();

        // Reset PID controllers
        positionController.reset();
        rotationController.reset();

        SmartDashboard.putString("Positioning Status", "In Progress");
    }

    /**
     * Stop the positioning process
     */
    public void stopPositioning() {
        targetTagOptions = null;
        targetTagId = null;
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0));

        SmartDashboard.putString("Positioning Status", "Completed");
    }

    /**
     * Calculate the position error between current position and desired position
     *
     * @param cameraToTarget The transform from camera to AprilTag
     * @return The transform from current position to desired position
     */
    private Transform2d calculatePositionError(Transform2d cameraToTarget) {
        // Calculate the transform from robot to AprilTag
        Transform2d robotToTarget = new Transform2d(
            cameraToTarget.getTranslation().plus(cameraToRobot.getTranslation().rotateBy(cameraToTarget.getRotation())),
            cameraToTarget.getRotation().plus(cameraToRobot.getRotation())
        );

        // Calculate the desired robot to target transform
        Transform2d desiredRobotToTarget = targetPoseRelative;

        // Calculate error (difference between current and desired)
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
        1.0, 0.0, new Rotation2d(Math.PI)
    );

    /**
     * Start aligning to an AprilTag that matches the ID given
     *
     * @param targetTagId The ID of the april tag to align to
     */
    public Command c_align(int targetTagId) {
        return c_align(new int[] { targetTagId });
    }

    /**
     * Start aligning to an AprilTag that matches the {@link TagGroup} given
     *
     * @param targetTagGroup A group of april tags to align to, e.g. {@code TagGroup.REEF}.
     *                       The robot will align to whatever one PhotonVision considers the best
     */
    public Command c_align(TagGroup targetTagGroup) {
        return c_align(tagIds.get(targetTagGroup));
    }

    /**
     * Start aligning to an AprilTag that matches one of the IDs given
     *
     * @param targetTagIds A list of april tag IDs to align to.
     *                     The robot will align to whatever one PhotonVision considers the best
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
}
