/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class AutoMagazine extends CommandBase {
    private Magazine magazine;
    
    // These are for adding callbacks when the intake and shooter sensors change, theres probably some slick way of doing this
    private boolean prevIntakeSensorOn;

    public AutoMagazine(Magazine magazine) {
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!magazine.getShooterSensor() && magazine.getIntakeSensor()) {
            magazine.setMagazineSpeed(-1);
        }
        else {
            // Turn magazine off when the intake sensor doesn't sense a ball
            if(prevIntakeSensorOn != magazine.getIntakeSensor()) {
                magazine.setMagazineSpeed(0);
                prevIntakeSensorOn = magazine.getIntakeSensor();
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
