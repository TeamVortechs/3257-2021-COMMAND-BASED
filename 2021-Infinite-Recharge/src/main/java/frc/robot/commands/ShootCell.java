package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShootCell extends SequentialCommandGroup {
    public ShootCell(Magazine magazine, Shooter shooter) {
        // Assuming the flywheel PID is enabled
        addCommands(
            new InstantCommand(() -> magazine.setMagazineSpeed(-1)),                                                    // Run Magazine
            new WaitUntilCommand(() -> magazine.getShooterSensor()).withTimeout(0.5),                                   // ...until ball is detected (or timeout is reached)
            new InstantCommand(() -> magazine.setEmpty(!magazine.getShooterSensor())),                                  // if the timeout is reached and there is no ball, tell the magazine its empty
            //new WaitUntilCommand(shooter.atSetpoint()).withTimeout(0.7),                                                // if there is a ball, wait 'til the shooter is up to speed
            new RunCommand(() -> magazine.setMagazineSpeed(-0.7)).withInterrupt(() -> !magazine.getShooterSensor())     // run the magazine until the ball is properly shot
        );
    }
}
