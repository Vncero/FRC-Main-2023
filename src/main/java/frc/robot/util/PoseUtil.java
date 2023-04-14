package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

import java.util.Collection;
import java.util.List;

public class PoseUtil {

    public static Pose2d averagePoses(Pose2d... poses) {
        return averagePoses(List.of(poses));
    }

    public static <T extends Collection<Pose2d>> Pose2d averagePoses(T poses) {

        Translation2d avgTranslation = poses.stream()
            .map(Pose2d::getTranslation)
            .reduce(new Translation2d(), Translation2d::plus)
            .div(poses.size());

        Rotation2d avgRotation = new Rotation2d(
                poses
                    .stream()
                    .map(p -> p.getRotation().getRadians())
                    .reduce(0.0, Double::sum)
        );

        return new Pose2d(avgTranslation, avgRotation);
    }

    public static <T extends Collection<Pose2d>> T sanitizePoses(T poses) {
        poses.removeIf(p -> !(
                p.getX() > Constants.GridAlign.kCamSanityXMin &&
                p.getX() < Constants.GridAlign.kCamSanityXMax &&
                p.getY() > Constants.GridAlign.kCamSanityZMin &&
                p.getY() < Constants.GridAlign.kCamSanityZMax
        ));

        return poses;
    }

    public static Sendable getDefaultPoseSendable(Pose2d pose) {
        return builder -> {
            builder.addDoubleProperty("Translation X", pose::getX, t -> {
            });
            builder.addDoubleProperty("Translation Z", pose::getY, t -> {
            });

            builder.addDoubleProperty("Rotation (degrees)", pose.getRotation()::getDegrees, t -> {
            });
            builder.addDoubleProperty("Rotation (radians)", pose.getRotation()::getRadians, t -> {
            });
        };
    }
}