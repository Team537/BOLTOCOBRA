package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Double LeftSpeed, RightSpeed;

    public DriveForwardCommand(DriveSubsystem driveSubsystem, Double LeftSpeed, Double RightSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.LeftSpeed = LeftSpeed;
        this.RightSpeed = RightSpeed;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveForwardCmd started!");
    }

    @Override
    public void execute() {
        driveSubsystem.setMotors(LeftSpeed, RightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        System.out.println("DriveForwardCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
