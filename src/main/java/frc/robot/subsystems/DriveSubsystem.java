package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import utils.AccelerationLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import utils.AccelerationLimiter;

public class DriveSubsystem extends SubsystemBase {
  //left
  private final WPI_TalonFX m_frontLeft = new WPI_TalonFX(DriveConstants.kFrontLeft);
  private final WPI_TalonFX m_rearLeft = new WPI_TalonFX(DriveConstants.kRearLeft);
  private final WPI_TalonFX m_frontRight = new WPI_TalonFX(DriveConstants.kFrontRight);
  //right
  private final WPI_TalonFX m_rearRight = new WPI_TalonFX(DriveConstants.kRearRight);


  // The motors on the left side of the drive.
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
  
  // The robot's drive
  // THE PROBLEM CHILD. This line being uncommented was causing our issues
 // private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
 private double m_left_setpoint, m_right_setpoint;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  // private final DifferentialDriveOdometry m_odometry;

  private boolean fastModeEnabled = true; 
  /** Creates a new DriveSubsystem. */


  public void motorDefaults(WPI_TalonFX m_motor, boolean inversion) {
    m_motor.configFactoryDefault(Constants.kTimeoutMs);
    m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);
    m_motor.setInverted(inversion);
    m_motor.setNeutralMode(NeutralMode.Coast);
    m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    m_motor.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_motor.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    m_motor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    m_motor.configMotionSCurveStrength(Constants.smoothing);
    m_motor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    m_motor.configMotionSCurveStrength(Constants.smoothing);
    m_motor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }
  public DriveSubsystem() {
    // m_frontLeft
    // m_frontRight
    // m_rearLeft
    // m_rearRight

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
    // m_frontLeft.setInverted(false);
    // m_frontRight.setInverted(true);
    // m_rearLeft.setInverted(false);
    // m_rearRight.setInverted(true);

    //bolt inverts
    motorDefaults(m_frontLeft, false);
    motorDefaults(m_frontRight, true);
    motorDefaults(m_rearLeft, false);
    motorDefaults(m_rearRight, true);


    //

    // 1. No effect
    
  // m_frontLeft.configAllowableClosedloopError(Constants.kSlotIdx, 5, Constants.kTimeoutMs);
  // m_frontRight.configAllowableClosedloopError(Constants.kSlotIdx, 5, Constants.kTimeoutMs);
  // m_rearLeft.configAllowableClosedloopError(Constants.kSlotIdx, 5, Constants.kTimeoutMs);
  // m_rearLeft.configAllowableClosedloopError(Constants.kSlotIdx, 5, Constants.kTimeoutMs)

  // m_frontLeft.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
  // m_frontRight.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
  // m_rearLeft.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
  // m_rearRight.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);

  // m_frontLeft.config_kP(Constants.kSlotIdx, .03, Constants.kTimeoutMs);
  // m_frontRight.config_kP(Constants.kSlotIdx, .03, Constants.kTimeoutMs);
  // m_rearLeft.config_kP(Constants.kSlotIdx, .03, Constants.kTimeoutMs);
  // m_rearRight.config_kP(Constants.kSlotIdx, .03, Constants.kTimeoutMs);

  // m_frontLeft.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
  // m_frontRight.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
  // m_rearLeft.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
  // m_rearRight.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);

  // m_frontLeft.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
  // m_frontRight.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
  // m_rearLeft.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
  // m_rearRight.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
  
  // m_frontLeft.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
	// m_frontRight.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
  // m_rearLeft.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
  // m_rearRight.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);

  // m_frontLeft.configMotionAcceleration(6000, Constants.kTimeoutMs);
	// m_frontRight.configMotionAcceleration(6000, Constants.kTimeoutMs);
  // m_rearLeft.configMotionAcceleration(6000, Constants.kTimeoutMs);
  // m_rearRight.configMotionAcceleration(6000, Constants.kTimeoutMs);

   // Sets the distance per pulse for the encoders
    // m_frontLeft.configPulseWidthPeriod_EdgesPerRot(4096, Constants.kTimeoutMs);
    // m_frontRight.configPulseWidthPeriod_EdgesPerRot(4096, Constants.kTimeoutMs);

    // resetEncoders();
    // m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    SmartDashboard.putData(this);
  }

  public void toggleFastMode() {
    fastModeEnabled = !fastModeEnabled;
  }
  @Override
  public void periodic() {
    // //Update the odometry in the periodic block
    // m_odometry.update(
    //     m_gyro.getRotation2d(), m_frontLeft.getSelectedSensorPosition(Constants.kPIDLoopIdx),
    //     m_frontRight.getSelectedSensorPosition(Constants.kPIDLoopIdx));

    SmartDashboard.putNumber("Front Left Motor Velocity", m_frontLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Front Right Motor Velocity", m_frontRight.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Rear Left Motor Velocity", m_rearLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Rear Right Motor Velocity", m_rearRight.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Front Left Motor Position", m_frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Front Right Motor Position", m_frontRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Left Motor Position", m_rearLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Rear Right Motor Position", m_rearRight.getSelectedSensorPosition());

    SmartDashboard.putNumber("Heading", m_gyro.getRotation2d().getDegrees());

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
  //  */
  // public Pose2d getPose() {
  // return m_odometry.getPoseMeters();
  // }

  // /**
  // * Returns the current wheel speeds of the robot.
  // *
  // * @return The current wheel speeds.
  // */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_frontLeft.getSelectedSensorVelocity(),
        m_frontRight.getSelectedSensorVelocity());
  }

  // /**
  // * Resets the odometry to the specified pose.
  // *
  // * @param pose The pose to which to set the odometry.
  // */
  public void resetOdometry(Pose2d pose) {
  resetEncoders();
  // m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDrive(double left, double right) {
    // m_drive.tankDrive(left, right);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(rightVolts);
    // m_frontRight.set(ControlMode.MusicTone, demand0, demand1Type, demand1);
    // m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    m_frontRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    m_rearLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    m_rearRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_frontRight.getSelectedSensorPosition(Constants.kPIDLoopIdx)) +
        (m_frontLeft.getSelectedSensorPosition(Constants.kPIDLoopIdx)) / 2.0;
  }

  // Called continuously when driving in auto or teleop
  public void setMotors(double leftSpeed, double rightSpeed) {
    // double clampedLeftSpeed = fastModeEnabled ? leftSpeed: leftSpeed;
    // double clampedRightSpeed = fastModeEnabled ? rightSpeed: rightSpeed;
    double speedMultiplyer = 0.5;

    m_frontLeft.set(leftSpeed * speedMultiplyer);
    m_frontRight.set(rightSpeed * speedMultiplyer);   //change speeds with multiplyers here
    m_rearLeft.set(leftSpeed * speedMultiplyer);
    m_rearRight.set(rightSpeed * speedMultiplyer);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void motion_magic_start_config_drive(boolean isForward, double lengthInTicks){
		m_left_setpoint = m_frontLeft.getSelectedSensorPosition() + lengthInTicks;
		m_right_setpoint = m_frontRight.getSelectedSensorPosition() + lengthInTicks;

		m_frontLeft.configMotionCruiseVelocity(12318, Constants.kTimeoutMs);
		m_frontLeft.configMotionAcceleration(6159, Constants.kTimeoutMs); //cruise velocity / 2, so will take 2 seconds
		m_frontRight.configMotionCruiseVelocity(12318, Constants.kTimeoutMs);
		m_frontRight.configMotionAcceleration(6159, Constants.kTimeoutMs);
    m_rearLeft.configMotionCruiseVelocity(12318, Constants.kTimeoutMs);
		m_rearLeft.configMotionAcceleration(6159, Constants.kTimeoutMs); //cruise velocity / 2, so will take 2 seconds
		m_rearRight.configMotionCruiseVelocity(12318, Constants.kTimeoutMs);
		m_rearRight.configMotionAcceleration(6159, Constants.kTimeoutMs);

		//set up talon to use DriveMM slots
	m_frontLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
  m_frontRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
  m_rearLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
  m_rearRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);


	
		// if(isForward == true){
		// 	m_left_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF);
		// 	m_right_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF);
		// } else{
		// 	m_left_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF * -1);
		// 	m_right_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF * -1);
		// }
	}

	public boolean motionMagicDrive(double target_position) {
		double tolerance = 1500;
		//add ifs if we need to set negative arbFF for going backward
		m_frontLeft.set(ControlMode.MotionMagic, m_left_setpoint); //, DemandType.ArbitraryFeedForward, arbFF);
		m_frontRight.set(ControlMode.MotionMagic, m_right_setpoint);//, DemandType.ArbitraryFeedForward, arbFF);
    m_rearLeft.set(ControlMode.MotionMagic, m_left_setpoint); //, DemandType.ArbitraryFeedForward, arbFF);
		m_rearRight.set(ControlMode.MotionMagic, m_right_setpoint);//, DemandType.ArbitraryFeedForward, arbFF);
		// m_left_leader.set(ControlMode.MotionMagic, target_position);
		// m_right_leader.set(ControlMode.MotionMagic, target_position);
	
		double currentPos_L = m_frontLeft.getSelectedSensorPosition();
		double currentPos_R = m_frontRight.getSelectedSensorPosition();
	
		return Math.abs(currentPos_L - m_left_setpoint) < tolerance && Math.abs(currentPos_R - m_right_setpoint) < tolerance;
	}



  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();

  }

  public void DriveSlow() { 
    m_frontLeft.set(TalonFXControlMode.PercentOutput, .25);
    m_frontRight.set(TalonFXControlMode.PercentOutput, .25);
    m_rearLeft.set(TalonFXControlMode.PercentOutput, .25);
    m_rearRight.set(TalonFXControlMode.PercentOutput, .25);
  }

  public void DriveFast() {
    m_frontLeft.set(TalonFXControlMode.PercentOutput, .55);
    m_frontRight.set(TalonFXControlMode.PercentOutput, .55);
    m_rearLeft.set(TalonFXControlMode.PercentOutput, .55);
    m_rearRight.set(TalonFXControlMode.PercentOutput, .55);
  }


  public void AutoBack() {
    m_frontLeft.set(TalonFXControlMode.PercentOutput, .2);
    m_frontRight.set(TalonFXControlMode.PercentOutput, .2);
    m_rearLeft.set(TalonFXControlMode.PercentOutput, .2);
    m_rearRight.set(TalonFXControlMode.PercentOutput, .2);
  }
  
  // public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
  //   m_leftLeader.setSetpoint(
  //       ExampleSmartMotorController.PIDMode.kPosition,
  //       left.position,
  //       m_feedforward.calculate(left.velocity));
  //   m_rightLeader.setSetpoint(
  //       ExampleSmartMotorController.PIDMode.kPosition,
  //       right.position,
  //       m_feedforward.calculate(right.velocity));
  // }

public void MagicTaxi(){ 
  
  m_frontLeft.set(TalonFXControlMode.MotionMagic, Constants.targetMeters);
	m_frontRight.set(TalonFXControlMode.MotionMagic, Constants.targetMeters);
  m_rearLeft.set(TalonFXControlMode.MotionMagic, Constants.targetMeters);
  m_rearRight.set(TalonFXControlMode.MotionMagic, Constants.targetMeters);

    
  }
}