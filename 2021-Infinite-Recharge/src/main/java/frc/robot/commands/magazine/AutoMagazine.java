package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class AutoMagazine extends CommandBase {
    private Magazine magazine;
    private Shooter shooter;
    
    // These are for adding callbacks when the intake and shooter sensors change, theres probably some slick way of doing this
    private boolean prevIntakeSensorOn;

    public AutoMagazine(Magazine magazine, Shooter shooter) {
        this.shooter = shooter;
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        if(!shooter.getShooting() && !magazine.getShooterSensor()) {        // if not shooting and not filled up
            if (magazine.getIntakeSensor()) {                               // succ if ball
                magazine.setMagazineSpeed(-1);
            }

            if(prevIntakeSensorOn != magazine.getIntakeSensor()) {
                if (magazine.getIntakeSensor()) {                           // the first frame it gets a ball
                    magazine.setEmpty(false);                               // let it know theres atleast one ball there
                }
                else {                                                      // the first frame a ball leaves
                    magazine.setMagazineSpeed(0);                           // turn off mag
                }

                prevIntakeSensorOn = magazine.getIntakeSensor();            // update
            }
        }
    }   

    @Override
    public void end(boolean interrupted) {
        magazine.setMagazineSpeed(0);                                       // turn off mag for safety
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
