package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public double targetX, targetY;
    public double targetArea;
    public NetworkTable tableInstance;
    public String name;

    public Limelight(String name) {
        this.name = name;
        tableInstance = NetworkTableInstance.getDefault().getTable(name);
        setLightState(2);
    }

    /**
     * Sets the limelights ledMode:
     * 0: Default to pipeline
     * 1: Force Off
     * 2: Force Blink
     * 3: Force On
     * @param mode to set the limelight leds
     */
    public void setLightState(int mode) {
        tableInstance.getEntry("ledMode").setNumber(mode);
    }
}
