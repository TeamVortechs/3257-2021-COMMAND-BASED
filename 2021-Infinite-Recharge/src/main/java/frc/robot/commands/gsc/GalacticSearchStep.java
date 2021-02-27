package frc.robot.commands.gsc;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GSCConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.RamseteHelper;

public class GalacticSearchStep extends SequentialCommandGroup {

    public GalacticSearchStep(BooleanSupplier condition, Runnable ballFoundRunnable, int step, Drivetrain drivetrain) {
        String stepName = GSCConstants.pathNames[step];
        addCommands(
            // Drive first search path
            RamseteHelper.fromPath(drivetrain, "/autonomous/GSC_Search" + step + ".wpilib.json"),

            // Check if ball
            new ConditionalCommand(
                new PrintCommand("PATH FOUND: " + stepName + "(Path "+ step + ")").andThen(() -> ballFoundRunnable.run()),
                new PrintCommand("The path isn't " + stepName + " (Path " + step + ")..."), 
                condition
            )
        );
    }
}
