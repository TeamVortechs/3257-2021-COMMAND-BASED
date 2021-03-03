package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class DumbAutoShoot extends SequentialCommandGroup {
    /**
     * Big sequential command to dumbly (not many sensors) shoot all the balls
     * there is literally no reason to use this lmao
     * @param shooter
     * @param magazine
     * @param flywheelRPM
     * @param intestineSpeed
     * @param pidTimeout
     */
    public DumbAutoShoot(Shooter shooter, Magazine magazine, double flywheelRPM, double intestineSpeed, double pidTimeout) {
        /**
         * Heres what it does: 
         *      enable and spin up the flywheel
         *      wait until the flywheel is up to speed (or if the timeout was reached)
         *      spin the belt for .5 seconds
         *      do it again 4 more times
         *      disable the flywheel
         * I'm hoping that with extra time I can get it to use the magazine sensors to help out with this
         */
        addCommands(
            /*new InstantCommand(() -> { 
                shooter.enable();
                shooter.setSetpoint(flywheelRPM);
            }),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(0))),
            new WaitUntilCommand(shooter.atSetpoint()).withTimeout(pidTimeout)
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(intestineSpeed)))
                .andThen(new WaitCommand(.5))
                .andThen(new InstantCommand(() -> magazine.setMagazineSpeed(0))),*/
            new InstantCommand(() -> { 
                //shooter.setSetpoint(0);
                //shooter.disable(); 
            })
        );
    }
}
