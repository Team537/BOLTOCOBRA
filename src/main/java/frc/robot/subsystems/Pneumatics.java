// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  PneumaticHub m_PneumaticHub = new PneumaticHub(1);  
  // private static int RforwardChannel =0;
  // private static int RreverseChannel =1;
  /** Creates a new Pneumatics. */
  Solenoid m_tshirtsolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
 // DoubleSolenoid m_RdoubleSolenoid = m_PneumaticHub.makeDoubleSolenoid(RforwardChannel,RreverseChannel);

  public Pneumatics() {


  }
  public void Neutral() {
    m_tshirtsolenoid.set(false);

  }
  public void OpenValve() {
    m_tshirtsolenoid.set(true);

  }
  public void CloseValve () {
    m_tshirtsolenoid.set(false);
  //  m_RdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
