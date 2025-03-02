package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.commands.NoOp;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    final SwerveSubsystem swerve;
    final PhotonCamera camera;
    final AprilTagFieldLayout fieldLayout;

    static final double OFFSET_X = 0.0;
    static final double OFFSET_Y = 0.0;

    public VisionSubsystem(SwerveSubsystem swerve, PhotonCamera camera) {
        this.swerve = swerve;
        this.camera = camera;

        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public Pose2d getPose(int side) {
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();

        if (target.getFiducialId() == 7) {
            Pose2d pose =
                fieldLayout.getTagPose(target.getFiducialId())
                    .get()
                    .toPose2d();

            Transform2d offset = new Transform2d(
                side * OFFSET_X, side * OFFSET_Y,
                Rotation2d.fromDegrees(0)
            );

            return pose.transformBy(offset);
        }

        return null;
    }

    public void updatePos() {
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();

        // TODO: set camera pos offset to center
        Pose3d pose = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            fieldLayout.getTagPose(target.getFiducialId()).get(),
            new Transform3d(1.0, 0.0, 0.0, null)
        );

        swerve.swerveDrive.addVisionMeasurement(
            pose.toPose2d(), Timer.getFPGATimestamp()
        );
    }

    public Command c_orient(int side) {
        Pose2d pose = getPose(side);

        if (pose != null) {
            return swerve.driveToPose(pose);
        } else {
            return new NoOp();
        }
    }
}
