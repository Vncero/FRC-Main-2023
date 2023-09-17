package frc.robot.subsystems.vision.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.VirtualSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import static frc.robot.Constants.Vision.*;

public class Vision extends VirtualSubsystem {
    private final PhotonCamera camera;

    private final PhotonPoseEstimator poseEstimator;

    private PhotonPipelineResult latestResult;

    private BiConsumer<PhotonPipelineResult, EstimatedRobotPose> visionDataConsumer = (result, pose) -> {
    };

    private Supplier<Pose2d> prevEstimateSupplier = Pose2d::new;

    public Vision() {
        this.camera = new PhotonCamera("limelight");

        this.poseEstimator = new PhotonPoseEstimator(
                kAprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                this.camera,
                kRobotToCameraTransform
        );
    }

    @Override
    public void virtualPeriodic() {
        PhotonPipelineResult latestResult = camera.getLatestResult();

        if (!latestResult.hasTargets()) return;

        this.latestResult = latestResult;

        poseEstimator.setReferencePose(prevEstimateSupplier.get());

        Optional<EstimatedRobotPose> pose = poseEstimator.update(latestResult);

        if (pose.isEmpty()) return;

        visionDataConsumer.accept(latestResult, pose.get());
    }

    public void setInterfaces(BiConsumer<PhotonPipelineResult, EstimatedRobotPose> visionDataConsumer,
                              Supplier<Pose2d> prevEstimateSupplier) {
        this.visionDataConsumer = visionDataConsumer;
        this.prevEstimateSupplier = prevEstimateSupplier;
    }

    // could be absolute nonsense
    public static Matrix<N3, N1> calculateStdDevs(PhotonPipelineResult result) {
        final double stdDev = result.getLatencyMillis() / result.getTargets().size();

        final double rotationStdDev = kRotationStdDevCoeff * stdDev;

        final Translation3d robotPose = new Translation3d();

        // weighted average by ambiguity
        final double avgTargetDistance = result.getTargets().stream().mapToDouble(t -> {
            final double bestCameraToTargetDistance = t.getBestCameraToTarget().getTranslation().getDistance(robotPose);
            final double altCameraToTargetDistance = t.getAlternateCameraToTarget().getTranslation().getDistance(robotPose);

            return t.getPoseAmbiguity() * (bestCameraToTargetDistance + altCameraToTargetDistance);
        }).reduce(0.0, Double::sum) / (2.0 * result.getTargets().size());

        final double translationStdDev = kTranslationStdDevCoeff * avgTargetDistance * stdDev;

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }
}