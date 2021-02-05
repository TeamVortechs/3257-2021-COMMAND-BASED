package frc.robot.commands;

import java.util.Hashtable;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Magazine;
import frc.robot.utils.RamseteHelper;

public class GalacticSearch extends SequentialCommandGroup {

    int chosenPath = -1;

    public GalacticSearch(Drivetrain drivetrain, Magazine magazine) {
        for (int i = 0; i < 3; i++) {
            final int step = i + 1;
            addCommands(
                new GalacticSearchStep(
                    () -> magazine.getIntakeLimelight().hasTarget(), 
                    () -> chosenPath = step,
                    step, 
                    drivetrain
                ).withInterrupt(() -> chosenPath != -1)
            );
        }
        addCommands(
            new ConditionalCommand(
                new PrintCommand("NO PATH FOUND: Using Path B Blue (Path 4)")
                    .andThen(() -> chosenPath = 4), 
                new PrintCommand("Running selected path..."), 
                () -> chosenPath == -1
            ),
            new InstantCommand(() -> magazine.setIntakeSpeed(.7)),
            RamseteHelper.fromPath(drivetrain, "/autonomous/GSC_Path" + chosenPath + ".wpilib.json"),
            new InstantCommand(() -> magazine.setIntakeSpeed(0)),
            new PrintCommand("If nothing went horribly wrong, I am at the endzone with all 3 balls!")
        );
    }

    public static Hashtable<Integer, String> stepToOutput = new Hashtable<Integer, String>() {
        private static final long serialVersionUID = -436855052785248771L;
        {
            put(1, "PATH B RED");
            put(2, "PATH A RED");
            put(3, "PATH A BLUE");
            put(4, "PATH B BLUE");
        }
    };
}