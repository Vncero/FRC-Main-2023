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

    private final Gyro gyro = new Gyro();
    private final Drivetrain drivetrain = new Drivetrain(gyro);
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final SideCamera sideCamera = new SideCamera(0, 1);

    private final PhotonLimelight photonLimelight = new PhotonLimelight("limelight");

    private final AutoPicker autoPicker;

    public RobotContainer() {
        this.autoPicker = new AutoPicker(drivetrain, arm, gyro, intake);

        this.drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverController));

        this.arm.setDefaultCommand(arm.runToSetpoints());

        this.intake.setDefaultCommand(intake.intakeOff());

        configureButtonBindings();
        doSendables();
    }

    private void configureButtonBindings() {
        // driver slow mode
        Controller.onHold(driverController.RightBumper, Commands.runOnce(() -> driverController.setSlowMode(Mode.SLOW)));
        Controller.onRelease(driverController.RightBumper, Commands.runOnce(() -> driverController.setSlowMode(Mode.NORMAL)));

        // driver intake
        Controller.onHold(driverController.RightTrigger, intake.runOnce(intake::intakeFast));
        Controller.onHold(driverController.LeftTrigger, intake.runOnce(intake::outtakeFast));
        // manipulator intake
        Controller.onHold(manipulatorController.intakeElementTriggerFast, intake.runOnce(intake::intakeFast));
        Controller.onHold(manipulatorController.outtakeElementTriggerFast, intake.runOnce(intake::outtakeFast));
        Controller.onHold(manipulatorController.intakeElementTriggerSlow, intake.runOnce(intake::intakeSlow));
        Controller.onHold(manipulatorController.outtakeElementTriggerSlow, intake.runOnce(intake::outtakeSlow));

        // manipulator toggle cube
        Controller.onPress(manipulatorController.RightBumper, Commands.runOnce(() -> this.arm.armMode = ArmMode.CUBE));
        // manipulator toggle cone
        Controller.onPress(manipulatorController.LeftBumper, Commands.runOnce(() -> this.arm.armMode = ArmMode.CONE));

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
    public CommandBase rumble(Controller controller, double time) {
        return Commands.startEnd(
            () -> controller.setRumble(true),
            () -> controller.setRumble(false)
        ).withTimeout(time).withName("Rumble (Time: " + time + "s)");
    }

    public CommandBase poleAlign(double setpoint) {
        return Commands.run(photonLimelight::update)
            // unfortunately, there is no syntactic sugar for this yet (if it gets too annoying, remove and only runOnce())
            .until(photonLimelight.getResult()::hasTargets).unless(photonLimelight.getResult()::hasTargets)
            .andThen(new PIDCommand(
                new PIDController(Constants.PoleAlign.kP, Constants.PoleAlign.kI, Constants.PoleAlign.kD),
                () -> photonLimelight.getResult().getBestTarget().getYaw(),
                () -> setpoint,
                outputPower -> drivetrain.arcadeDrive(0, -outputPower),
                drivetrain
            )).withName("PoleAlign (Setpoint: " + setpoint + ")");
    }
}