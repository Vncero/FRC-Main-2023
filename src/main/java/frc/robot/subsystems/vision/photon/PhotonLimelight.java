package frc.robot.subsystems.vision.photon;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

public class PhotonLimelight implements Sendable {
    private PhotonCamera limelight;

    private PhotonPoseEstimator poseEstimator;

    private PhotonPipelineResult result; 

    public PhotonLimelight(String cameraName) {
        this.limelight = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(
            Constants.Vision.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP,
            limelight,
            Constants.Vision.kRobotToCameraTransform
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return this.poseEstimator.update();
    }

    public void update() {
        result = this.limelight.getLatestResult(); 
    }

    public Pose3d getRelativePoint(Pose3d pose) {
        Optional<EstimatedRobotPose> optPose = getEstimatedRobotPose(); 

        if (optPose.isEmpty()) return null;  

        EstimatedRobotPose unpacked = optPose.get(); 

        return pose.relativeTo(unpacked.estimatedPose); 
    }

    public Pose3d getAprilTag(int fiducialID) {
        if (result == null || !result.hasTargets()) return null; 
        List<PhotonTrackedTarget> targets = result.targets; 
        for (PhotonTrackedTarget t : targets) {
            if (t.getFiducialId() != fiducialID) continue;
            return new Pose3d().transformBy(Constants.Vision.kRobotToCameraTransform).transformBy(t.getBestCameraToTarget());
        }
        return null;
    }

    public Pose2d getAprilTag2d(int fiducialID) {
        return getAprilTag(fiducialID).toPose2d(); 
    }

    public Pose2d getRelativePoint2d(Pose2d pose) {
        Optional<EstimatedRobotPose> optPose = getEstimatedRobotPose(); 

        if (optPose.isEmpty()) return null;  

        EstimatedRobotPose unpacked = optPose.get(); 

        return pose.relativeTo(unpacked.estimatedPose.toPose2d()); 
    }

    public PhotonPipelineResult getResult() {
        return this.result;
    }

    // TODO: finish or scrap for choosers
    @Override
    public void initSendable(SendableBuilder builder) {

    }
}