package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.Arm.ScoringPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnBy;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.intake.Intake;
import org.bananasamirite.robotmotionprofile.Waypoint;

public class AutoPicker {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final Drivetrain drivetrain;
    private final Arm arm;
    private final Intake intake;
    private final Gyro gyro;

    public AutoPicker(Drivetrain drivetrain, Arm arm, Gyro gyro, Intake intake) {
        this.drivetrain = drivetrain; 
        this.arm = arm; 
        this.gyro = gyro; 
        this.intake = intake;
        
        configureAutos();
    }

    private void configureAutos() {
        autoChooser.addOption("Red No Bump Lane", topLaneAuto(ScoringPosition.HIGH_CONE, ScoringPosition.HIGH_CUBE, Alliance.Red));
        autoChooser.addOption("Blue No Bump Lane", topLaneAuto(ScoringPosition.HIGH_CONE, ScoringPosition.HIGH_CUBE, Alliance.Blue));
        autoChooser.addOption("Balance Lane Cone", midLaneAuto(ScoringPosition.HIGH_CONE));
        autoChooser.addOption("Balance Lane Cube", midLaneAuto(ScoringPosition.HIGH_CUBE));
        autoChooser.addOption("Taxi Balance Lane Cone", midLaneAutoTaxi(ScoringPosition.HIGH_CONE));
        autoChooser.addOption("Taxi Balance Lane Cube", midLaneAutoTaxi(ScoringPosition.HIGH_CUBE));
        autoChooser.addOption("Red Bump Lane Cone", bottomLaneAuto(ScoringPosition.HIGH_CONE));
        autoChooser.addOption("Red Bump Lane Cube", bottomLaneAuto(ScoringPosition.HIGH_CUBE));
        autoChooser.addOption("Blue Bump Lane Cone", bottomLaneAuto(ScoringPosition.HIGH_CONE));
        autoChooser.addOption("Blue Bump Lane Cube", bottomLaneAuto(ScoringPosition.HIGH_CUBE));
    }

    public SendableChooser<Command> getAutoChooser() {
        return this.autoChooser; 
    }

    public CommandBase balance(double setpoint) {
        return new PIDCommand(
            new PIDController(Constants.Balance.kP, Constants.Balance.kI, Constants.Balance.kD),
            gyro::getPitch,
            () -> setpoint,
            outputPower -> drivetrain.arcadeDrive(-outputPower, 0),
            drivetrain
        ).withName("Balance (Setpoint: " + setpoint + ")"); // PIDCommand does everything Balance did by default (except accepting negative output)
    }
    public CommandBase score(ScoringPosition scoringPosition) {
        return arm.moveToPos(scoringPosition.getPosition()).andThen(
            Commands.waitSeconds(0.25),
            intake.outtakeElement(scoringPosition.getSpeed()),
            arm.runOnce(() -> {
                arm.anchorSetpoint = Constants.Arm.Position.CONTRACTED.getAnchor();
                arm.floatingSetpoint = Constants.Arm.Position.CONTRACTED.getFloating();
            })
        ).withName("Score (" + scoringPosition + ")");
    }

    public CommandBase topLaneAuto(ScoringPosition scoreFirst, ScoringPosition scoreSecond, Alliance alliance) {
        final double allianceMultiplier = alliance == Alliance.Red ? 1 : -1;

        final Waypoint OUT_FIELD = new Waypoint(-4.28, -3.31 * allianceMultiplier, 0, 1.5, 0.5);
        // increase 7.34 -> increase dist from grid
        final Waypoint PRE_TOP_PIECE = new Waypoint(-7.28, -2.0 * allianceMultiplier, Math.toRadians(-90 * allianceMultiplier), 0.5, 1);
        // increase 7.34 -> increase dist from grid
        final Waypoint TOP_PIECE = new Waypoint(-7.28, -3 * allianceMultiplier, Math.toRadians(-90 * allianceMultiplier), 0.6, 1);

        final Waypoint startWaypoint;
        final Waypoint endWaypoint;

        switch (scoreFirst) {
            case HIGH_CUBE:
            case MID_CUBE:
            case LOW_CUBE:
                // -3.64 -> -3.60
                startWaypoint = new Waypoint(-1.86, -3.68 * allianceMultiplier, 0, 1, 1);
                break;
            case HIGH_CONE:
            case MID_CONE:
            case LOW_CONE:
            default:
                startWaypoint = new Waypoint(-1.86, -3.06 * allianceMultiplier, 0, 1, 1);
                break;
        }

        switch (scoreSecond) {
            case HIGH_CUBE:
            case MID_CUBE:
            case LOW_CUBE:
                // -3.64 -> -3.60
                endWaypoint = new Waypoint(-1.8, -3.68 * allianceMultiplier, 0, 1, 1); // TODO: tune this
                break;
            case HIGH_CONE:
            case MID_CONE:
            case LOW_CONE:
            default:
                endWaypoint = new Waypoint(-1.8, -3.06 * allianceMultiplier, 0, 1, 1);
                break;
        }

        return score(scoreFirst).andThen(
            Commands.waitUntil(drivetrain.gyro._gyro::isConnected),
            Constants.Trajectory.trajectoryCreator.createCommand(drivetrain, new Waypoint[] {
                    startWaypoint,
                    OUT_FIELD,
                    PRE_TOP_PIECE,
            }, new TrajectoryConfig(Constants.Trajectory.kMaxSpeedMetersPerSecond, Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared).setReversed(true), true)
                .alongWith(Commands.waitSeconds(0.75).andThen(arm.moveToPos(Constants.Arm.Position.GROUND))),
            Constants.Trajectory.trajectoryCreator.createCommand(drivetrain, new Waypoint[] {
                    PRE_TOP_PIECE,
                    TOP_PIECE,
                    OUT_FIELD,
                    endWaypoint
            }, new TrajectoryConfig(Constants.Trajectory.kMaxSpeedMetersPerSecond, Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared).setReversed(false), false)
                .alongWith(intake.intakeFor(Constants.Intake.ScoreSpeed.FAST, 2.0).andThen(arm.moveToPos(Constants.Arm.Position.CONTRACTED))),
            score(scoreSecond),
            Constants.Trajectory.trajectoryCreator.createCommand(drivetrain, new Waypoint[] {
                    endWaypoint,
                    OUT_FIELD,
                    PRE_TOP_PIECE
            }, new TrajectoryConfig(Constants.Trajectory.kMaxSpeedMetersPerSecond, Constants.Trajectory.kMaxAccelerationMetersPerSecondSquared).setReversed(true), false)
        ).withName("TopLaneAuto (1. " + scoreFirst + ", 2. " + scoreSecond + ")");
    }

    public CommandBase midLaneAuto(ScoringPosition position) {
        return score(position).andThen(
            Commands.waitUntil(drivetrain.gyro._gyro::isConnected),
            drivetrain.moveBackward(0.5),
            new TurnBy(drivetrain, 180).withTimeout(2.0),
            drivetrain.run(() -> drivetrain.arcadeDrive(0.3, 0)).raceWith(
                Commands.waitUntil(() -> Math.abs(gyro.getPitch()) > 12)
                    .andThen(Commands.waitSeconds(1.0))
                    .withTimeout(2.5)
            ),
            balance(0)
        ).withName("MidLaneAuto (" + position + ")");
    }

    public CommandBase midLaneAutoTaxi(ScoringPosition position) {
        return score(position).andThen(
            Commands.waitUntil(drivetrain.gyro._gyro::isConnected),
            drivetrain.moveBackward(0.5),
            new TurnBy(drivetrain, 180).withTimeout(1.5),
            drivetrain.run(() -> drivetrain.arcadeDrive(0.65, 0)).until(() -> Math.abs(gyro.getPitch()) > 11).withTimeout(2.5),
            drivetrain.run(() -> drivetrain.arcadeDrive(0.24, 0)).raceWith(
                Commands.waitUntil(() -> Math.abs(gyro.getPitch()) < 4).andThen(Commands.waitSeconds(0.55))
            ),
            drivetrain.runOnce(() -> drivetrain.arcadeDrive(0, 0)),
            Commands.waitSeconds(0.65),
            drivetrain.run(() -> drivetrain.arcadeDrive(-0.31, 0)).raceWith(
                Commands.waitUntil(() -> Math.abs(gyro.getPitch()) > 12).andThen(Commands.waitSeconds(1.0))
            ),
            balance(0)
        ).withName("MidLaneAutoTaxi (" + position + ")");
    }

    public CommandBase bottomLaneAuto(ScoringPosition position) {
        return score(position).andThen(
            drivetrain.moveBackward(5.0),
            Commands.waitUntil(drivetrain.gyro._gyro::isConnected),
            drivetrain.turnToAngle(180).withTimeout(2.0)
        ).withName("BottomLaneAuto (" + position + ")");
    }
}