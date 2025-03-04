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
        BARGE,
        COMPACTOR
    }

    private static final int TAGS_PER_FIELD_SIDE = 11;
    private static final HashMap<TagGroup, int[]> tagIds = new HashMap<>();

    static {
        // -1 means any tag if in first item of array
        tagIds.put(TagGroup.ANY, new int[] { -1 });
        tagIds.put(TagGroup.INTAKE, new int[] { 1, 2 });
        tagIds.put(TagGroup.REEF, new int[] { 6, 7, 8, 9, 10, 11 });
        tagIds.put(TagGroup.BARGE, new int[] { 4, 5 });
        tagIds.put(TagGroup.COMPACTOR, new int[] { 3 });

        // move april tags to other side of board if on the blue alliance
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

    // pid controllers
    private final PIDController positionController;
    private final PIDController rotationController;

    // target pose relative to tag
    private Transform2d targetPoseRelative;

    // timeout for positioning
    private final double TIMEOUT_SECONDS = 3.0;
    private double startTime = 0;

    // ids for all of the possible tags to target
    private int[] targetTagOptions = null;

    // int if aligning to a certain tag, null if not
    private Integer targetTagId = null;

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
        rotationController = new PIDController(1.0, 0.0, 0.0);

        // make rotation controller continuous
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // set tolerances
        positionController.setTolerance(POS_TOLERANCE_METERS);
        rotationController.setTolerance(Math.toRadians(ROT_TOLERANCE_DEG));

        // target pose relative to tag
        // TODO change depending on selection
        targetPoseRelative = new Transform2d(0, 0, Rotation2d.kZero);
    }

    @Override
    public void periodic() {
        if (!this.isPositioning()) return;

        if (targetTagId == null) {
            // find best tag out of possible and target it
            targetTagId = getBestTargetId(targetTagOptions);
        }

        // get latest result from photonvision
        PhotonTrackedTarget target = targetTagId != null ? getTarget(targetTagId) : null;

        if (target == null) {
            // stop movement since no tags are visible
            swerveDrive.drive(new ChassisSpeeds(0, 0, 0));

            // stop positioning if tag has not been seen for long time
            if (Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS) {
                stopPositioning();
                SmartDashboard.putString("Positioning Status", "Failed - No AprilTag visible");
            }

            return;
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

        // apply deadbands and clamp values for safety
        xSpeed = applyDeadband(xSpeed, 0.05);
        ySpeed = applyDeadband(ySpeed, 0.05);
        rotSpeed = applyDeadband(rotSpeed, 0.05);

        // max speeds
        // TODO tune speeds
        double maxLinearSpeed = 15.0;  // meters per second
        double maxRotSpeed = Math.PI; // radians per second

        xSpeed = Util.clamp(xSpeed, -maxLinearSpeed, maxLinearSpeed);
        ySpeed = Util.clamp(ySpeed, -maxLinearSpeed, maxLinearSpeed);
        rotSpeed = Util.clamp(rotSpeed, -maxRotSpeed, maxRotSpeed);

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
        SmartDashboard.putNumber("Position Error X", desiredOff.getX());
        SmartDashboard.putNumber("Position Error Y", desiredOff.getY());
        SmartDashboard.putNumber("Rotation Error (deg)", Math.toDegrees(desiredOff.getRotation().getRadians()));

        // check if reached target position
        boolean atPosition =
            Math.abs(desiredOff.getX()) < POS_TOLERANCE_METERS &&
            Math.abs(desiredOff.getY()) < POS_TOLERANCE_METERS &&
            Math.abs(desiredOff.getRotation().getDegrees()) < ROT_TOLERANCE_DEG;

        // check if timed out
        boolean timedOut = Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS;

        if (atPosition || timedOut) {
            stopPositioning();
        }
    }

    /**
     * Find the best April Tag to target given a list of April Tag IDs
     *
     * @param tagIds The IDs of the April Tags we want to target
     * @return The ID of the best April Tag, or null if none were found
     */
    private Integer getBestTargetId(int[] tagIds) {
        PhotonPipelineResult result = photonCamera.getLatestResult();

        if (!result.hasTargets()) return null;

        // -1 represents any, so return any best result
        if (tagIds[0] == -1) {
            return result.getTargets().getFirst().fiducialId;
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
        this.startTime = Timer.getFPGATimestamp();

        // reset pid controllers
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
        1.0, 0.0, new Rotation2d(Math.PI)
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
}
