package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.poseTracker.PoseTracker;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.MoveToPos;
import frc.robot.subsystems.arm.commands.RunToSetpoints;
import frc.robot.subsystems.drivetrain.commands.TeleopDrive;
import frc.robot.util.Controller;
import frc.robot.util.ControllerUtils;
import frc.robot.util.DriverController;
import frc.robot.util.ManipulatorController;
import frc.robot.util.DriverController.Mode;
import org.bananasamirite.robotmotionprofile.Waypoint;

public class RobotContainer {
  /* Controllers */
  private final DriverController driverController = new DriverController(0);
  private final ManipulatorController manipulatorController = new ManipulatorController(1);

  /* Subsystems */
  private Drivetrain drivetrain = new Drivetrain();
  private Arm arm = new Arm();
  private Intake intake = new Intake();
  private Gyro gyro = new Gyro();
  private PoseTracker poseTracker = new PoseTracker(drivetrain);
    
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final RamseteCommand command;

  public RobotContainer() {
    this.drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driverController));

    this.arm.setDefaultCommand(new RunToSetpoints(arm));

    this.intake.setDefaultCommand(new RunCommand(intake::off, intake));
//    command = new MotionProfileCommand(drivetrain, new TankMotionProfile(ParametricSpline.fromWaypoints(new Waypoint[] {
//      new Waypoint(0, 0, 0, 1, 1),
//      new Waypoint(2, 1, Math.toRadians(90), 1, 1)
//    }), ProfileMethod.TIME, new TankMotionProfileConstraints(1, 0.2)));
    command = Constants.Trajectory.trajectoryCreator.createCommand(drivetrain,
            new Waypoint[] {
                    new Waypoint(0, 0, 0, 1, 1),
                    new Waypoint(2, 0, Math.toRadians(0), 1, 1)
            }, 1, 0.2, false);

    // this.poseTracker.setDefaultCommand(new PrintCommand("Matt likes balls idk, Raf too"));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // intake
    Controller.onHold(driverController.RightTrigger, new RunCommand(intake::forward, intake));
    Controller.onHold(driverController.LeftTrigger, new RunCommand(intake::backward, intake));
    Controller.onHold(manipulatorController.intakeElementTrigger, new RunCommand(intake::forward));
    Controller.onHold(manipulatorController.outtakeElementTrigger, new RunCommand(intake::backward));

    // contract
    Controller.onPress(manipulatorController.A, new MoveToPos(arm, Constants.Arm.Positions.Contracted.kAnchor, Constants.Arm.Positions.Contracted.kFloating));
    // ground
    Controller.onPress(manipulatorController.B, new MoveToPos(arm, Constants.Arm.Positions.Ground.kAnchor, Constants.Arm.Positions.Ground.kFloating));

    // dynamic for tuning
    SmartDashboard.putNumber("anchor-setpoint", 13.0);
    SmartDashboard.putNumber("floating-setpoint", 22.0);
    Controller.onPress(manipulatorController.X, new MoveToPos(
      arm,
      () -> ControllerUtils.clamp(SmartDashboard.getNumber("anchor-setpoint", 0.0), 13.0, 90.0),
      () -> ControllerUtils.clamp(SmartDashboard.getNumber("floating-setpoint", 0.0), 22.0, 90.0)
    ));

    //slow mode
    Controller.onHold(driverController.RightBumper, new InstantCommand(() -> driverController.setSlowMode(Mode.SLOW)));
    Controller.onRelease(driverController.RightBumper, new InstantCommand(() -> driverController.setSlowMode(Mode.NORMAL)));
  }

  public Command getAutonomousCommand() {
    // return new InstantCommand(() -> {});
    return command;
  }

  public void doSendables() {
  }
}