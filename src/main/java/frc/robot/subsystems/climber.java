// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase {
  /** Creates a new climber. */
  private CANSparkMax m_climb = new CANSparkMax(Constants.ClimberConstants.kClimb, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerClimb = m_climb.getPIDController();
  private RelativeEncoder m_encoderClimb = m_climb.getEncoder();
  private CANSparkMax m_climb2 = new CANSparkMax(Constants.ClimberConstants.kClimb2, MotorType.kBrushless);
  private SparkMaxPIDController m_pidControllerClimb2 = m_climb2.getPIDController();
  private RelativeEncoder m_encoderClimb2 = m_climb2.getEncoder();
  public climber() {
   // m_climb2.setInverted(true);

  }

   

  // public void climberIdle() {m_pidControllerClimb.setP(Constants.ClimberConstants.kP);
  //   m_pidControllerClimb.setI(Constants.ClimberConstants.kI);
  //   m_pidControllerClimb.setD(Constants.ClimberConstants.kD);
  //   m_pidControllerClimb.setIZone(Constants.ClimberConstants.kIz);
  //   m_pidControllerClimb.setFF(Constants.ClimberConstants.kFF);
  //   m_pidControllerClimb.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
  //   m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
  //   m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
  //   m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
  //   m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
  //   m_pidControllerClimb.setReference(Constants.ClimberConstants.kRotationsDown, CANSparkMax.ControlType.kSmartMotion);

  //   m_pidControllerClimb2.setI(Constants.ClimberConstants.kI);
  //   m_pidControllerClimb2.setD(Constants.ClimberConstants.kD);
  //   m_pidControllerClimb2.setIZone(Constants.ClimberConstants.kIz);
  //   m_pidControllerClimb2.setFF(Constants.ClimberConstants.kFF);
  //   m_pidControllerClimb2.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
  //   m_pidControllerClimb2.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
  //   m_pidControllerClimb2.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
  //   m_pidControllerClimb2.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
  //   m_pidControllerClimb2.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
  //   m_pidControllerClimb2.setReference(Constants.ClimberConstants.kRotationsDown, CANSparkMax.ControlType.kSmartMotion);
  // }



  public void climberUp() {
    m_pidControllerClimb.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb.setReference(Constants.ClimberConstants.kLeftRotationsUp, CANSparkMax.ControlType.kSmartMotion);

    m_pidControllerClimb2.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb2.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb2.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb2.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb2.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb2.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb2.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb2.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb2.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb2.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb2.setReference(Constants.ClimberConstants.kRightRotationsUp, CANSparkMax.ControlType.kSmartMotion);
  }



  public void climberDown() {
    m_pidControllerClimb.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb.setReference(Constants.ClimberConstants.kLeftRotationsDown, CANSparkMax.ControlType.kSmartMotion);

    m_pidControllerClimb2.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb2.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb2.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb2.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb2.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb2.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb2.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb2.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb2.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb2.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb2.setReference(Constants.ClimberConstants.kRightRotationsDown, CANSparkMax.ControlType.kSmartMotion);
  } 

  public void moveclimber(int climbercount){
    m_pidControllerClimb.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb.setReference(climbercount, CANSparkMax.ControlType.kSmartMotion);

    m_pidControllerClimb2.setP(Constants.ClimberConstants.kP);
    m_pidControllerClimb2.setI(Constants.ClimberConstants.kI);
    m_pidControllerClimb2.setD(Constants.ClimberConstants.kD);
    m_pidControllerClimb2.setIZone(Constants.ClimberConstants.kIz);
    m_pidControllerClimb2.setFF(Constants.ClimberConstants.kFF);
    m_pidControllerClimb2.setOutputRange(Constants.ClimberConstants.kMinOutput, Constants.ClimberConstants.kMaxOutput);
    m_pidControllerClimb2.setSmartMotionMaxVelocity(Constants.ClimberConstants.kMaxV, 0);
    m_pidControllerClimb2.setSmartMotionMinOutputVelocity(Constants.ClimberConstants.kMinV, 0);
    m_pidControllerClimb2.setSmartMotionMaxAccel(Constants.ClimberConstants.kMaxA, 0);
    m_pidControllerClimb2.setSmartMotionAllowedClosedLoopError(Constants.ClimberConstants.kAllE, 0);
    m_pidControllerClimb2.setReference(climbercount, CANSparkMax.ControlType.kSmartMotion);
  }






  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber", m_encoderClimb.getPosition());
    SmartDashboard.putNumber("Right Climber", m_encoderClimb2.getPosition());
  }
}
