package frc.robot.commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MotionMagicDrive extends CommandBase {
  /** Creates a new DriveMM. */
  DriveSubsystem m_driveTrain;
  double m_targetPosition;
  int count;
  //the number of times motion magic is on target before the command finishes
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  public MotionMagicDrive(DriveSubsystem driveTrain, double targetMeters) {
    m_driveTrain = driveTrain;
    // Distance in inches to ticks conversion is:
    /*
		   wheelRotations = positionMeters/(2 * Math.PI * kWheelRadiusInches);
	   	 motorRotations = wheelRotations * kSensorGearRatio
		   sensorCounts =   motorRotations * kCountsPerRev
     */
    m_targetPosition = (targetMeters/0.4787787204060999) * (.555556 * 2048);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
    count = 0;
    m_driveTrain.motion_magic_start_config_drive(m_targetPosition >= 0, m_targetPosition);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.motionMagicDrive(m_targetPosition)){
      count++;
    } else {
      count = 0;
    }
    SmartDashboard.putNumber("THE SPECIAL NUMER THING", count);
    SmartDashboard.putNumber("Distance in Meters",  (m_driveTrain.getAverageEncoderDistance() * .4787787204060999) / (2048 * .5555556));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (m_driveTrain.getAverageEncoderDistance() )
    return count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}