package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class BasicAuto extends CommandBase {
  /**
   * Creates a new DriveAuto.
   */

   long timeCurrent = 0;
  DriveSubsystem driveSubsystem;

  public BasicAuto(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(this.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timeCurrent = System.currentTimeMillis();
    driveSubsystem.arcadeDrive(0.5, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
    // driveSubsystem.arcadeDrive(0.4, 0);

//   }

  // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     driveSubsystem.tankDrive(0, 0);
//   }

  // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     //   return true;
//       return this.timeCurrent > 10000;
//   }
}