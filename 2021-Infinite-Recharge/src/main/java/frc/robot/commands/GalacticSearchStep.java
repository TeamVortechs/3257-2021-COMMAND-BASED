/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.RamseteHelper;

public class GalacticSearchStep extends SequentialCommandGroup {

    public GalacticSearchStep(BooleanSupplier condition, Runnable ballFoundRunnable, int step, Drivetrain drivetrain) {
        String stepName = GalacticSearch.stepToOutput.get(step);
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
