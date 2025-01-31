package frc.robot.subsystems.vision.limelight;

//import java.net.URISyntaxException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.util.enums.CamMode;
import frc.robot.util.enums.Displacement;
import frc.robot.util.enums.LedMode;
import frc.robot.util.enums.Snapshot;
import frc.robot.util.enums.StreamMode;

/* vision TODO:
*   - mess with web interface & make pipeline(s) for detecting game pieces (cube, cone)
*   - research/attempt switching from mjpeg to h.264 for camera streaming (potential-engine)
*/

public class LimelightAPI {

    private static final NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");

    // public Websocket wsClient;

    public static boolean logging;

//    public LimelightAPI(boolean logging) throws URISyntaxException {
        // this.wsClient = new Websocket(new URI(Constants.Limelight.kLimelightURLString));
        // this.logging = logging;
        // LimelightAPI.limelightNT =
        // NetworkTableInstance.getDefault().getTable("limelight");

        // if (LimelightAPI.limelightNT == null) {
        // SmartDashboard.putString("Limelight table", "null");
        // }

        // SmartDashboard.putString("Limelight table", "not null");
//    }

    // public Pose2d getActualPose2d() {
    //     var rawJson = this.wsClient.getMessage();

    //     if (rawJson == null) {
    //         System.out.println("rawJson is null");
    //         return new Pose2d();
    //     }

    //     try {

    //         ObjectMapper mapper = new ObjectMapper();
    //         JsonNode node = mapper.readTree(rawJson);
    //         System.out.println("node" + node);

    //         return new Pose2d();

    //         // double tx = node.get("transform").get("tx").asDouble();
    //         // double tz = node.get("transform").get("tz").asDouble();
    //         // double ry = node.get("transform").get("ry").asDouble();

    //         // return new Pose2d(tx, tz, new Rotation2d(ry * Math.PI / 180));

    //     } catch (JsonMappingException e) {
    //         System.out.println("fuck1");
    //         System.out.println(e);
    //     } catch (JsonProcessingException e) {
    //         System.out.println("fuck2");
    //         System.out.println(e);
    //     }
    //     return new Pose2d();

    // }

    public static void logPoses(Pose3d camPose, Pose3d botPose) {

        SmartDashboard.putNumber("campose x (z adj)", camPose.getX() / camPose.getZ());
        SmartDashboard.putNumber("campose y (z adj)", camPose.getY() / camPose.getZ());
        SmartDashboard.putNumber("campose z", camPose.getZ());

        /*
         * SmartDashboard.putNumber("adj campose x",
         * Limelight.adjustCamPose(camPose).getX());
         * SmartDashboard.putNumber("adj campose y",
         * Limelight.adjustCamPose(camPose).getY());
         * SmartDashboard.putNumber("adj campose z",
         * Limelight.adjustCamPose(camPose).getZ());
         */

        SmartDashboard.putNumber("campose rX", camPose.getRotation().getX());
        SmartDashboard.putNumber("campose rY", camPose.getRotation().getY());
        SmartDashboard.putNumber("campose rZ", camPose.getRotation().getZ());

        SmartDashboard.putNumber("botpose X", botPose.getX());
        SmartDashboard.putNumber("botpose y", botPose.getY());
        SmartDashboard.putNumber("botpose z", botPose.getZ());

        SmartDashboard.putNumber("botpose rX", botPose.getRotation().getX());
        SmartDashboard.putNumber("botpose rY", botPose.getRotation().getY());
        SmartDashboard.putNumber("botpose rZ", botPose.getRotation().getZ());
    }

    /* Returns an adjusted Pose2D based on camera pose */
    public static Pose2d adjustCamPose(Displacement displacement) {
        Pose2d camPose = getCamPoseTargetSpace();

        double dZ = camPose.getY() + 0.69 * 0.420; //nice.
        double dX = camPose.getX() + displacement.getOffset();

        double actualRot = Math.signum(dX) * camPose.getRotation().getRadians();

        Pose2d camPose2 = getTargetPoseRobotSpace();

        SmartDashboard.putNumber("frfr rot", Math.signum(-camPose2.getX()) * Math.abs(camPose2.getRotation().getDegrees()));
        SmartDashboard.putNumber("frfr sideways", -camPose2.getX());
        SmartDashboard.putNumber("frfr forward", camPose2.getY());

        double adjustedRot = Math.atan2(-dX, -dZ);

        double theta = adjustedRot - actualRot;

        double distance = Math.hypot(dX, dZ);

        double adjustedX = (distance * Math.cos(theta)) - Constants.GridAlign.kAdjustZ * Math.cos(actualRot);
        double adjustedZ = (-(distance * Math.sin(theta)) - Constants.GridAlign.kAdjustZ * Math.sin(actualRot));

        return new Pose2d(adjustedX, adjustedZ, new Rotation2d(actualRot));
    }

    // TODO: questionable code, perhaps test??
    public static Pose2d getOffsetTargetPose(double epsilon, double delta) { // x, y offsets (with respect to target, Cartesian)
        Pose2d targetPose = getTargetPoseRobotSpace();
        Pose2d camPose = getCamPoseTargetSpace(); // target space

        double theta = camPose.getRotation().getRadians();
        double alpha = targetPose.getRotation().getRadians();

        double phi = -Math.signum(camPose.getX()) * (Math.PI - (theta + alpha));

        // to reinterpret epsilon & delta in robot space, rotate by phi, the angle between the axes of target & robot space
        // as a side effect, the coordinates are switched i.e. horizontal motion in robot space is y as opposed to x (and vice versa, more or less)
        // could perhaps use Pose2d#relativeTo()
        double rotatedEpsilon = epsilon * Math.cos(phi) - delta * Math.sin(phi);
        double rotatedDelta = epsilon * Math.sin(phi) + delta * Math.cos(phi);

        double adjustedX = targetPose.getX() + rotatedEpsilon;
        double adjustedY = targetPose.getY() + rotatedDelta;

        // as a final touch, adjust sign of horizontal motion to accommodate for transition between Limelight robot space and WPILib robot space
        return new Pose2d(adjustedX, -adjustedY, targetPose.getRotation());
    }

    public static boolean hasValidTargets() {
        return LimelightAPI.limelightNT.getEntry("tv").getDouble(0) == 1;
    }

    public static double getHorizontalOffset() {
        return LimelightAPI.limelightNT.getEntry("tx").getDouble(0);
    }

    public static double getVerticalOffset() {
        return LimelightAPI.limelightNT.getEntry("ty").getDouble(0);
    }

    public static double getSkew() {
        return LimelightAPI.limelightNT.getEntry("ts").getDouble(0);
    }

    public static double getTargetArea() {
        return LimelightAPI.limelightNT.getEntry("ta").getDouble(0);
    }

    public static double getLatency() {
        return LimelightAPI.limelightNT.getEntry("tl").getDouble(0);
    }

    public static double getShortSideLength() {
        return LimelightAPI.limelightNT.getEntry("tshort").getDouble(0);
    }

    public static double getLongSideLength() {
        return LimelightAPI.limelightNT.getEntry("tlong").getDouble(0);
    }

    public static double getVerticalSideLength() {
        return LimelightAPI.limelightNT.getEntry("tvert").getDouble(0);
    }

    public static double getHorizontalSideLength() {
        return LimelightAPI.limelightNT.getEntry("thor").getDouble(0);
    }

    public static double getAprilTagID() {
        return LimelightAPI.limelightNT.getEntry("tid").getDouble(0);
    }

    public static double getPipeline() {
        return LimelightAPI.limelightNT.getEntry("getpipe").getDouble(0);
    }

    public static Object rawJSONTargets() {
        return LimelightAPI.limelightNT.getEntry("json").getValue().getValue();
    }

    public static void setPipeline(int pipeline) {
        if (pipeline > 9 || pipeline < 0) {
            SmartDashboard.putString("Limelight pipeline", "invalid pipeline");
        } else {
            LimelightAPI.limelightNT.getEntry("pipeline").setNumber(pipeline);
            SmartDashboard.putString("Limelight pipeline", "pipeline set to" + pipeline);
        }
    }

    public static void setLEDMode(LedMode mode) {
        LimelightAPI.limelightNT.getEntry("ledMode").setNumber(mode.getValue());
    }

    public static void setCamMode(CamMode mode) {
        LimelightAPI.limelightNT.getEntry("camMode").setNumber(mode.getValue());
    }

    public static void setStreamMode(StreamMode mode) {
        LimelightAPI.limelightNT.getEntry("stream").setNumber(mode.getValue());
    }

    public static void setSnapshotMode(Snapshot mode) {
        LimelightAPI.limelightNT.getEntry("snapshot").setNumber(mode.getValue());
    }

    public static void sendToRobot(double[] llrobot) {
        LimelightAPI.limelightNT.getEntry("llrobot").setDoubleArray(llrobot);
    }

    public static double[] getFromRobot() {
        double[] llpython = new double[6];
        return LimelightAPI.limelightNT.getEntry("llpython").getDoubleArray(llpython);
    }

    public static void setLogging(boolean logging) {
        LimelightAPI.logging = logging;
    }

    public static Pose3d getPose(String target) {
        double[] smd = new double[6];

        double[] poseRaw = LimelightAPI.limelightNT.getEntry(target).getDoubleArray(smd);

        if (poseRaw.length != 6) {
            return new Pose3d();
        }

        for (int i = 3; i < poseRaw.length; i++) {
            poseRaw[i] *= (Math.PI / 180);
        }
        Rotation3d rotationPose = new Rotation3d(poseRaw[5], poseRaw[3], poseRaw[4]);

        return new Pose3d(poseRaw[0], poseRaw[1], poseRaw[2], rotationPose);
    }

    public static Pose2d getPose(String pose, String space) {
        return flattenPose(getPose(pose + "_" + space));
    }

    public static Pose2d flattenPose(Pose3d raw) {
        return new Pose2d(raw.getX(), raw.getZ(), new Rotation2d(raw.getRotation().getY()));
//        return raw.toPose2d();
    }

    // target pose
    public static Pose2d getTargetPoseRobotSpace() {
        return getPose("targetpose", "robotspace");
    }

    public static Pose2d getTargetPoseCameraSpace() {
        return getPose("targetpose", "cameraspace");
    }

    // TODO: upload .fmap to web interface for field space
    // bot pose
    public static Pose2d getBotPoseFieldSpace() {
        return flattenPose(getPose("botpose"));
    }

    public static Pose2d getBotPoseWPIRedSpace() {
        return getPose("botpose", "wpired");
    }

    public static Pose2d getBotPoseWPIBlueSpace() {
        return getPose("botpose", "wpiblue");
    }

    public static Pose2d getBotPoseTargetSpace() {
        return getPose("botpose", "targetspace");
    }

    public static Pose2d getCamPoseRobotSpace() {
        return getPose("camerapose", "robotspace");
    }

    public static Pose2d getCamPoseTargetSpace() {
        return getPose("camerapose", "targetspace");
    }
}