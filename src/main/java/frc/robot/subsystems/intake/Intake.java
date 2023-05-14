package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    public CommandBase intakeOff() {
        return runOnce(this::off).withName("IntakeOff");
    }

    public CommandBase intakeFor(ScoreSpeed speed, double seconds) {
        return startEnd(speed == ScoreSpeed.FAST ? this::intakeFast : this::intakeSlow, this::off).withTimeout(seconds)
//                Commands.either(runOnce(this::intakeFast), runOnce(this::intakeSlow), () -> speed == ScoreSpeed.FAST)
//                   .andThen(Commands.waitSeconds(seconds))
//                   .andThen(intakeOff()).
            .withName("IntakeFor (Speed: " + speed + ", Time: " + seconds + "s)");
    }

    public CommandBase outtakeFor(ScoreSpeed speed, double seconds) {
        return startEnd(speed == ScoreSpeed.FAST ? this::outtakeFast : this::outtakeSlow, this::off).withTimeout(seconds)
//                Commands.either(runOnce(this::outtakeFast), runOnce(this::outtakeSlow), () -> speed == ScoreSpeed.FAST)
//                   .andThen(Commands.waitSeconds(seconds))
//                   .andThen(intakeOff())
            .withName("OuttakeFor (Speed: " + speed + ", Time: " + seconds + "s)");
    }

    public CommandBase intakeElement(ScoreSpeed speed) {
        return intakeFor(speed, Constants.Intake.kAutoIntakeSeconds).withName("IntakeElement (Speed" + speed + ")");
    }

    public CommandBase outtakeElement(ScoreSpeed speed) {
        return outtakeFor(speed, Constants.Intake.kAutoOuttakeSeconds).withName("OuttakeElement (Speed" + speed + ")");
    }
}