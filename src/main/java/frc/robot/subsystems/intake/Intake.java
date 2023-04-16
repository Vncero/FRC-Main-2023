package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.ScoreSpeed;

public class Intake extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(Constants.Intake.kIntakePort, MotorType.kBrushless);
    private DigitalInput beamBreak = new DigitalInput(Constants.Intake.kBeambreakPort); 

    public double power = 0;
    
    public Intake() {
        this.motor.setIdleMode(IdleMode.kBrake);
        this.motor.setSmartCurrentLimit(30);

        // SmartDashboard.putNumber("intake-fast", SmartDashboard.getNumber("intake-fast", Constants.Intake.kHighPower));
        // SmartDashboard.putNumber("intake-slow", SmartDashboard.getNumber("intake-slow", Constants.Intake.kLowPower));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("has element", hasElement()); 
        this.motor.set(power);
    }

    public void off() {
        this.power = 0;
    }

    public void outtakeFast() {
        this.power = -Constants.Intake.kHighPower;
    }

    public void outtakeSlow() {
        this.power = -Constants.Intake.kLowPower;
    }

    public void intakeFast() {
        this.power = Constants.Intake.kHighPower;
    }

    public void intakeSlow() {
        this.power = Constants.Intake.kLowPower;
    }

    public boolean hasElement() {
        return beamBreak.get(); 
    }

    // considering making the above methods for setting power return commands, gets repetitive to use runOnce() every time
    // with these command factories they're never used without being wrapped in a command
    public Command ensureOff() {
        return runOnce(this::off).withName("EnsureOff");
    }

    public Command intakeFor(ScoreSpeed speed, double seconds) {
        return Commands.either(runOnce(this::intakeFast), runOnce(this::intakeSlow), () -> speed == ScoreSpeed.FAST)
                   .andThen(Commands.waitSeconds(seconds))
                   .andThen(ensureOff()).withName("IntakeFor");
    }

    public Command outtakeFor(ScoreSpeed speed, double seconds) {
        return Commands.either(runOnce(this::outtakeFast), runOnce(this::outtakeSlow), () -> speed == ScoreSpeed.FAST)
                   .andThen(Commands.waitSeconds(seconds))
                   .andThen(ensureOff()).withName("OuttakeFor");
    }

    public Command intakeElement(ScoreSpeed speed) {
        return intakeFor(speed, Constants.Intake.kAutoIntakeSeconds).withName("IntakeElement");
    }

    public Command outtakeElement(ScoreSpeed speed) {
        return outtakeFor(speed, Constants.Intake.kAutoOuttakeSeconds).withName("OuttakeElement");
    }
}