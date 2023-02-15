//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Conveyer extends SubsystemBase { 
  private final WPI_TalonFX m_conveyer = new WPI_TalonFX(Constants.ConveyerConstants.kConveyer);


  /** Creates a new Intake. */
  public Conveyer() {
    
  //m_Intake.configFactoryDefault(Constants.kTimeoutMs);
  m_conveyer.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void conveyorForward() {
    m_conveyer.set(0.4);
  }
  public void conveyorOff() {
    m_conveyer.set(0.0);
  }
  public void conveyorBack() {
    m_conveyer.set(-0.4);
  }

}