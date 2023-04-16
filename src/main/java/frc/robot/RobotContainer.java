package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sidecamera.SideCamera;
import frc.robot.subsystems.vision.photon.PhotonLimelight;
import frc.robot.util.Controller;
import frc.robot.util.DriverController;
import frc.robot.util.DriverController.Mode;
import frc.robot.util.ManipulatorController;
import frc.robot.util.enums.ArmMode;

public class RobotContainer {
  private final DriverController driverController = new DriverController(0);
  private final ManipulatorController manipulatorController = new ManipulatorController(1);

  private Gyro gyro = new Gyro();
  private Drivetrain drivetrain = new Drivetrain(gyro);
  private Arm arm = new Arm();
  private Intake intake = new Intake();
  private SideCamera sideCamera = new SideCamera(0, 1);

  private PhotonLimelight photonLimelight = new PhotonLimelight("limelight");

  private final AutoPicker autoPicker;

  public RobotContainer() {
    this.autoPicker = new AutoPicker(drivetrain, arm, gyro, intake); 

    this.drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverController));

    this.arm.setDefaultCommand(arm.runToSetpoints());

    this.intake.setDefaultCommand(intake.ensureOff());

    configureButtonBindings();
    doSendables();
  }

  private void configureButtonBindings() {
    // driver slow mode
    Controller.onHold(driverController.RightBumper, new InstantCommand(() -> driverController.setSlowMode(Mode.SLOW)));
    Controller.onRelease(driverController.RightBumper, new InstantCommand(() -> driverController.setSlowMode(Mode.NORMAL)));

    // driver intake
    Controller.onHold(driverController.RightTrigger, new RunCommand(intake::intakeFast, intake));
    Controller.onHold(driverController.LeftTrigger, new RunCommand(intake::outtakeFast, intake));
    // manipulator intake
    Controller.onHold(manipulatorController.intakeElementTriggerFast, new RunCommand(intake::intakeFast));
    Controller.onHold(manipulatorController.outtakeElementTriggerFast, new RunCommand(intake::outtakeFast));
    Controller.onHold(manipulatorController.intakeElementTriggerSlow, new RunCommand(intake::intakeSlow));
    Controller.onHold(manipulatorController.outtakeElementTriggerSlow, new RunCommand(intake::outtakeSlow));

    // manipulator toggle cube
    Controller.onPress(manipulatorController.RightBumper, new InstantCommand(() -> {
      this.arm.armMode = ArmMode.CUBE;
    }));
    // manipulator toggle cone
    Controller.onPress(manipulatorController.LeftBumper, new InstantCommand(() -> {
      this.arm.armMode = ArmMode.CONE;
    }));

    /*
      Manipulator Arm State
    */
    // ground
    Controller.onPress(manipulatorController.A, arm.moveToPos(Constants.Arm.Position.GROUND));
    // contract
    Controller.onPress(manipulatorController.B, arm.moveToPos(Constants.Arm.Position.CONTRACTED));
    // mid - 
    Controller.onPress(manipulatorController.X, arm.moveToPos(arm.armMode == ArmMode.CUBE ? Constants.Arm.Position.MID_CUBE : Constants.Arm.Position.MID_CONE));
    // high
    Controller.onPress(manipulatorController.Y, arm.moveToPos(arm.armMode == ArmMode.CUBE ? Constants.Arm.Position.HIGH_CUBE : Constants.Arm.Position.HIGH_CONE));
    
    // station
    Controller.onPress(driverController.A, arm.moveToPos(Constants.Arm.Position.STATION));
    // shelf
    Controller.onPress(driverController.B, arm.moveToPos(Constants.Arm.Position.SHELF));
    // contract
    Controller.onPress(driverController.X, arm.moveToPos(Constants.Arm.Position.CONTRACTED));


    // TESTING
    // Controller.onPress(driverController.Y, new ParallelRaceGroup(
    //   new TurnBy(drivetrain, 90),
    //   new WaitCommand(3.0)
    // ));
  }

  public Command getAutonomousCommand() {
    return this.autoPicker.getAutoChooser().getSelected(); 
  }

  public void doSendables() {
    SmartDashboard.putData(this.autoPicker.getAutoChooser());

    SmartDashboard.putBoolean("Navex sucks", !gyro._gyro.isConnected());
  }

      // could put this into Controller/ControllerUtils
      public Command rumble(Controller controller, double time) {
          return Commands.startEnd(
              () -> controller.setRumble(true),
              () -> controller.setRumble(false)
          ).withTimeout(time).withName("Rumble");
      }

      public Command balance(double setpoint) {
          return new PIDCommand(
              new PIDController(Constants.Balance.kP, Constants.Balance.kI, Constants.Balance.kD),
              gyro::getPitch,
              () -> setpoint,
              outputPower -> drivetrain.arcadeDrive(-outputPower, 0),
              drivetrain
          ).withName("Balance"); // PIDCommand does everything Balance did by default (except accepting negative output)
      }

      public Command poleAlign(double setpoint) {
          return Commands.run(photonLimelight::update)
              // unfortunately, there is no syntactic sugar for this yet (if it gets too annoying, remove and only runOnce())
              .until(photonLimelight.getResult()::hasTargets).unless(photonLimelight.getResult()::hasTargets)
              .andThen(new PIDCommand(
                  new PIDController(Constants.PoleAlign.kP, Constants.PoleAlign.kI, Constants.PoleAlign.kD),
                  () -> photonLimelight.getResult().getBestTarget().getYaw(),
                  () -> setpoint,
                  outputPower -> drivetrain.arcadeDrive(0, -outputPower),
                  drivetrain
          )).withName("PoleAlign");
      }

      public Command score(Constants.Arm.ScoringPosition scoringPosition) {
          return arm.moveToPos(scoringPosition.getPosition())
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(intake.outtakeElement(scoringPosition.getSpeed()))
                    .finallyDo(interrupt -> {
                        arm.anchorSetpoint = Constants.Arm.Position.CONTRACTED.getAnchor();
                        arm.floatingSetpoint = Constants.Arm.Position.CONTRACTED.getFloating();
                    });
      }
}