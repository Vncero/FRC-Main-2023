package frc.robot.subsystems.poseTracker;

import java.util.ArrayList;
import java.util.List;

import org.bananasamirite.robotmotionprofile.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.PipelineIndex;
import frc.robot.util.PoseUtil;
import frc.robot.util.SizedQueue;
import frc.robot.util.enums.Displacement;
import frc.robot.util.limelight.LimelightAPI;

public class PoseTracker extends SubsystemBase {
        // Getting last 3 camera pose values
        private SizedQueue<Pose2d> camPoseQueue = new SizedQueue<Pose2d>(3);

        // Getting last 3 bot pose values
        private SizedQueue<Pose2d> botPoseQueue = new SizedQueue<Pose2d>(3);

    private Pose2d avgPythonCamPose;

        private Pose2d avgAprilTagCamPose;

    public Displacement displacement = Displacement.kCenter;

    public PoseTracker() {}

    @Override
    public void periodic() {
        // setting the last 3
        this.camPoseQueue.add(LimelightAPI.adjustCamPose(this.displacement));

        this.botPoseQueue.add(LimelightAPI.adjustCamPose(this.displacement));

        this.avgAprilTagCamPose =  getAverageAprilPose();

        SmartDashboard.putNumber("avg campose x", avgAprilTagCamPose.getX());
        SmartDashboard.putNumber("avg campose z", avgAprilTagCamPose.getY());
        SmartDashboard.putNumber("avg rotation", avgAprilTagCamPose.getRotation().getDegrees());
    }

        // TODO: are we scrapping this? definitely something to discuss
        public Pose2d getSensorFusionAverage() {
                return PoseUtil.averagePipelinePoses(new ArrayList<>(List.of(avgAprilTagCamPose, avgPythonCamPose)));
        }

        public void clearAndSetPipeline(PipelineIndex index) {
                this.camPoseQueue.clear();
                this.botPoseQueue.clear();

                LimelightAPI.setPipeline(index.getValue());
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