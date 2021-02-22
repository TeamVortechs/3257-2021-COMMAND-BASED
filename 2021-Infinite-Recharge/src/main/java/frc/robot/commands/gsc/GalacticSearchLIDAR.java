package frc.robot.commands.gsc;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
            new InstantCommand(() -> chosenPath = DeterminePath()),
            RamseteHelper.fromPath(drivetrain, String.format("/autonomous/full/GSC_Search%d.wpilib.json", chosenPath)),
            new PIDCommand(
                new PIDController(0.7, 0.1, 0), 
                ()->(drivetrain.getLeftEncoderPosition() + drivetrain.getRightEncoderPosition())/2,
                () -> stepToOutput.get(chosenPath).y, 
                (output) -> {
                    System.out.println((drivetrain.getLeftEncoderPosition()+drivetrain.getRightEncoderPosition())/2);
                    drivetrain.tankDrive(-output, -output);
                }, drivetrain)
        );
        // FOR PATH 1 (GSC_B_Red1) full throttle 2.4 meters
        // FOR PATH 2 (GSC_A_Red2) full throttle 3.4 meters
        // FOR PATH 3 (GSC_A_Blue3) full throttle 1.5 meters
    }

    private int DeterminePath() {
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

        if (minDiff < 10) {
            System.out.println("PATH FOUND: " + stepToOutput.get(selectedIndex+1).x + ", " + minDiff + "cm OFF");
            return selectedIndex;
        } else {
            System.out.println("NOT ALIGNED GOOD, NEAREST PATH: " + stepToOutput.get(selectedIndex+1).x + ", " + minDiff + "cm OFF");
            return -1;
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
    public static Hashtable<Integer, Tuple<String, Double>> stepToOutput = new Hashtable<Integer, Tuple<String, Double>>() {
        private static final long serialVersionUID = -436855052785248771L;
        {
            put(1, new Tuple<String, Double>("GSC_B_Red1", 2.4));
            put(2, new Tuple<String, Double>("GSC_A_Red2", 3.4));
            put(3, new Tuple<String, Double>("GSC_A_Blue3", 1.5));
            put(4, new Tuple<String, Double>("GSC_B_Blue4", 0.0));
        }
    };
}
