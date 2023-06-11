package frc.robot.subsystems.vision.photon;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.VirtualSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public class PhotonLimelight extends VirtualSubsystem {
    private final PhotonCamera limelight;

    private final PhotonPoseEstimator poseEstimator;

    private PhotonPipelineResult result;

    public PhotonLimelight(String cameraName) {
        this.limelight = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(
            Constants.Vision.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP,
            limelight,
            Constants.Vision.kRobotToCameraTransform
        );

        updateResult();
    }

    @Override
    public void virtualPeriodic () {
        SmartDashboard.putBoolean("LL-isConnected", limelight.isConnected());
        SmartDashboard.putBoolean("LL-driverMode", limelight.getDriverMode());
    }

    private void updateResult() {
        var latestResult = limelight.getLatestResult();

        if (!latestResult.hasTargets()) return;

        if (!latestResult.getTargets().equals(result.getTargets())) result = latestResult;
    }

    public List<PhotonTrackedTarget> getTrackedTargets() {
        return this.result.getTargets();
    }

    // drivetrain should update these using odo
    /**
     * @return pose of target relative to robot (for pathing)
     */
    public Optional<Pose2d> getTargetPose2d() {

    }

    /**
     * @param
     * @return pose of target relative to robot (for arm/lift kinematics)
     */
    public Optional<Pose3d> getTargetPose3d(int tagId) {

    }

    public Optional<EstimatedRobotPose> getPoseEstimate() {

    }

//    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
//        return this.poseEstimator.update();
//    }
//
//    public void update() {
//        result = this.limelight.getLatestResult();
//    }
//
//    public Pose3d getRelativePoint(Pose3d pose) {
//        Optional<EstimatedRobotPose> optPose = getEstimatedRobotPose();
//
//        if (optPose.isEmpty()) return null;
//
//        EstimatedRobotPose unpacked = optPose.get();
//
//        return pose.relativeTo(unpacked.estimatedPose);
//    }
//
//    public Pose3d getAprilTag(int fiducialID) {
//        if (result == null || !result.hasTargets()) return null;
//        List<PhotonTrackedTarget> targets = result.targets;
//        for (PhotonTrackedTarget t : targets) {
//            if (t.getFiducialId() != fiducialID) continue;
//            return new Pose3d().transformBy(Constants.Vision.kRobotToCameraTransform).transformBy(t.getBestCameraToTarget());
//        }
//        return null;
//    }
//
//    public Pose2d getAprilTag2d(int fiducialID) {
//        return getAprilTag(fiducialID).toPose2d();
//    }
//
//    public Pose2d getRelativePoint2d(Pose2d pose) {
//        Optional<EstimatedRobotPose> optPose = getEstimatedRobotPose();
//
//        if (optPose.isEmpty()) return null;
//
//        EstimatedRobotPose unpacked = optPose.get();
//
//        return pose.relativeTo(unpacked.estimatedPose.toPose2d());
//    }
}