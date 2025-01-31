package frc.robot.subsystems.vision.poseTracker;

import org.bananasamirite.robotmotionprofile.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.limelight.LimelightAPI;
import frc.robot.util.PoseUtil;
import frc.robot.util.SizedQueue;
import frc.robot.util.enums.Displacement;

public class PoseTracker extends SubsystemBase {
    // Getting last 3 camera pose values
    private final SizedQueue<Pose2d> camPoseQueue = new SizedQueue<>(3);

    // Getting last 3 bot pose values
    private final SizedQueue<Pose2d> botPoseQueue = new SizedQueue<>(3);

    private Pose2d avgPythonCamPose;

    private Pose2d avgAprilTagCamPose;

    public Displacement displacement = Displacement.kCenter;

    @Override
    public void periodic() {
        // setting the last 3
        this.camPoseQueue.add(LimelightAPI.adjustCamPose(this.displacement));

        this.botPoseQueue.add(LimelightAPI.adjustCamPose(this.displacement));

        this.avgAprilTagCamPose = getAverageAprilPose();

        SmartDashboard.putNumber("avg campose x", avgAprilTagCamPose.getX());
        SmartDashboard.putNumber("avg campose z", avgAprilTagCamPose.getY());
        SmartDashboard.putNumber("avg rotation", avgAprilTagCamPose.getRotation().getDegrees());
    }

    // TODO: are we scrapping this? definitely something to discuss
    public Pose2d getSensorFusionAverage() {
        return PoseUtil.averagePoses(avgAprilTagCamPose, avgPythonCamPose);
    }

    public Pose2d getAverageAprilPose() {
        // return LimelightAPI.adjustCamPose();
        return PoseUtil.averagePoses(this.camPoseQueue);
    }

    public Waypoint[] generateWaypoints() {
        Pose2d pose = this.getAverageAprilPose();

        double relativeDistance = Math.hypot(pose.getX(), pose.getY());

        double weight = Constants.GridAlign.kGridWeight * relativeDistance;

        Waypoint[] waypoints = {
            new Waypoint(0, 0, 0, weight, 1),
            new Waypoint(pose.getX(), pose.getY(), pose.getRotation().getRadians(), weight, 1)
        };

        return waypoints;
    }
}