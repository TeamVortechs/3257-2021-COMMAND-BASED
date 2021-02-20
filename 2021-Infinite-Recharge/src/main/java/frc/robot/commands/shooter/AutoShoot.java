package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends SequentialCommandGroup {
    /**
     * Sequential group to shoot with many sensor inputs (limelights, magazine breakbeams, PIDs, etc)
     * 
     * @param shooter
     * @param magazine
     * @param flywheelRPM
     */
    public AutoShoot(Shooter shooter, Magazine magazine) {
        /**
         * heres what it does: 
         *      enable and spin up the flywheel
         *      do this:
         *          run the magazine forward until a ball is ready to be shot
         *          if it runs for 0.5 second and there is no ball (mag is empty), disable the shooter and end the entire shoot command
         *          the flywheel shouldve been spinning up during that time, wait until its up to speed (or 0.7 seconds have passed and its probably good enough)
         *          run the magazine until the ball is gone
         *      until the magazine is empty
         *      disable the flywheel, turn off shooting
         */
        addCommands(
            new InstantCommand(() -> { 
                //shooter.enable();                                                               // Enable and start shooter
                //shooter.setSetpoint(shooter.getCurrentTargetRPM());                             // Set shooter rpm
                shooter.setShooting(true);                                                      // Set mag shooting mode (disable auto intake)
            }),
            new ShootCell(magazine, shooter).withInterrupt(() -> magazine.getEmpty()),          // Shoot a power cell
            new ShootCell(magazine, shooter).withInterrupt(() -> magazine.getEmpty()),          // Shoot a power cell
            new ShootCell(magazine, shooter).withInterrupt(() -> magazine.getEmpty()),          // Shoot a power cell
            new ShootCell(magazine, shooter).withInterrupt(() -> magazine.getEmpty()),          // Shoot a power cell
            new ShootCell(magazine, shooter).withInterrupt(() -> magazine.getEmpty()),          // Shoot a power cell
            new InstantCommand(() -> {                                                          // Disable shooter
                //shooter.disable();
                shooter.setShooting(false);
            })
        );
    }
}
