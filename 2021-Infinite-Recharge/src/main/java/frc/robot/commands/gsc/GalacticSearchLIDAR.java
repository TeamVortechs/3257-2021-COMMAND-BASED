package frc.robot.commands.gsc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GSCConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Magazine;
import frc.robot.utils.RamseteHelper;

public class GalacticSearchLIDAR extends SequentialCommandGroup {
    Magazine magazine;
    int chosenPath = -1;
    public GalacticSearchLIDAR(Magazine magazine, Drivetrain drivetrain) {
        this.magazine = magazine;
        addCommands(
            new InstantCommand(() -> {
                chosenPath = DeterminePath(magazine, false);
                magazine.setIntakeSpeed(0.4);
                magazine.setMagazineSpeed(-0.7);
            }),
            RamseteHelper.fromPath(drivetrain, String.format("/autonomous/full/"+GSCConstants.pathNames[chosenPath]+".wpilib.json", chosenPath)),
            new InstantCommand(() -> {
                magazine.setIntakeSpeed(0);
                magazine.setMagazineSpeed(0);
                drivetrain.resetOdometry();
            })
        );
    }

    public static int DeterminePath(Magazine magazine, boolean picky) {
        double distance = magazine.getMagazineLidarDist();

        double minDiff = 0;
        int selectedIndex = 0;
        for(int i = 0; i < GSCConstants.pathDists.length; i++){
            double localDiff = GSCConstants.pathDists[i] - distance;
            if(localDiff < minDiff){
                selectedIndex = i;
                minDiff = localDiff;
            }
        }
        if (picky) {
            if (minDiff < 10) {
                System.out.println("PATH FOUND: " + GSCConstants.pathNames[selectedIndex] + ", " + minDiff + "cm OFF");
                return selectedIndex;
            } else {
                System.out.println("NOT ALIGNED GOOD, NEAREST PATH: " + GSCConstants.pathNames[selectedIndex] + ", " + minDiff + "cm OFF");
                return -1;
            }
        }else{
            return selectedIndex;
        }
    }
    public static class Tuple<X, Y> { 
        public final X x; 
        public final Y y; 
        public Tuple(X x, Y y) { 
          this.x = x; 
          this.y = y; 
        } 
    }
}
