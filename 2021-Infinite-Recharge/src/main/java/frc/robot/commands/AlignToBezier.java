package frc.robot.commands;

import java.awt.geom.Point2D;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.BezierCurve;

public class AlignToBezier extends CommandBase 
{
    DoubleSupplier headingSupplier;
    BezierCurve curve;
    double angle = 0;
    double speed;
    Drivetrain drivetrain;
    
    public AlignToBezier(double speed, BezierCurve curve, double t, Drivetrain drivetrain) 
    {
        addRequirements(drivetrain);
        this.curve = curve;
        this.angle = curve.getAngle(t);
        this.speed = speed;
        this.drivetrain = drivetrain;
    }

    public AlignToBezier(double speed, Point2D end, Point2D control1, Point2D control2, double t, Drivetrain drivetrain)
    {
        addRequirements(drivetrain);
        this.curve = new BezierCurve(control1, control2, end);
        this.angle = curve.getAngle(t);
        this.speed  = speed;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() 
    {
        headingSupplier = drivetrain::getHeading;
        System.out.print("Aligning to: " + angle);
    }

    @Override
    public void execute() 
    {
        double turningValue = Math.copySign(speed, angle - drivetrain.getHeading()); // use pid loop shitter
        drivetrain.arcadeDrive(0, -turningValue);
    }

    @Override
    public boolean isFinished() 
    {
        return Math.abs(angle - drivetrain.getHeading()) < DriveConstants.turnPIDTolerance;
    }

    @Override
    public void end(boolean interrupted) 
    {
        drivetrain.tankDrive(0, 0);
    }
}