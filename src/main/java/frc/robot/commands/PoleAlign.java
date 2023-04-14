package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.limelight.LimelightAPI;

public class PoleAlign extends PIDCommand {
    public PoleAlign(Drivetrain drivetrain) {
        this(drivetrain, 0);
    }

    public PoleAlign(Drivetrain drivetrain, double setpoint) {
        super(
            new PIDController(Constants.PoleAlign.kP, Constants.PoleAlign.kI, Constants.PoleAlign.kD),
            LimelightAPI::getHorizontalOffset,
                () -> setpoint,
            output -> drivetrain.arcadeDrive(0, output),
            drivetrain
        );
    }
}