package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.MoveToPos;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeElement;

public class Score extends SequentialCommandGroup {
    public Score(Arm arm, Intake intake, Constants.Arm.Position position) {
        addCommands(new MoveToPos(arm, position), new OuttakeElement(intake));
    }
}
