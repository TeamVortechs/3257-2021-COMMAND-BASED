package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// dont mess with
public final class Main {
  private Main() {}
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
