package frc.robot.commands.magazine;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.Timer;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MagazineConstants;
import frc.robot.subsystems.Magazine;

public class AutoMagazine extends CommandBase {
    private Magazine magazine;
    private PowerDistributionPanel pdp;
    private boolean prevIntakeSensorOn;

    public AutoMagazine(Magazine magazine, PowerDistributionPanel pdp) {
        this.magazine = magazine;
        this.pdp = pdp;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        if (!magazine.getShooterSensor()) {
            if (prevIntakeSensorOn != magazine.getIntakeSensor()) {
                if (magazine.getIntakeSensor()) {
                    magazine.setMagazineSpeed(-1);
                }
                else {
                    Timer timer = new Timer(200, (ActionListener) new ActionListener() {
                        @Override
                        public void actionPerformed(ActionEvent arg0) {
                            magazine.setMagazineSpeed(0);
                        }
                    });
                    timer.setRepeats(false);
                    timer.start();
                }
                prevIntakeSensorOn = magazine.getIntakeSensor();
            }

            if (pdp.getCurrent(MagazineConstants.intakePowerChannel) > 100) {
                magazine.setMagazineSpeed(-1);
            }
        }
    }   

    @Override
    public void end(boolean interrupted) {
        magazine.setMagazineSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
