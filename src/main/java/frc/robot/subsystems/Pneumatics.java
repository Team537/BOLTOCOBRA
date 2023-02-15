// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  PneumaticHub m_PneumaticHub = new PneumaticHub(11);  
  private static int LforwardChannel =14;
  private static int LreverseChannel =15;
  // private static int RforwardChannel =0;
  // private static int RreverseChannel =1;
  /** Creates a new Pneumatics. */
  DoubleSolenoid m_LdoubleSolenoid = m_PneumaticHub.makeDoubleSolenoid(LforwardChannel,LreverseChannel);
 // DoubleSolenoid m_RdoubleSolenoid = m_PneumaticHub.makeDoubleSolenoid(RforwardChannel,RreverseChannel);

  public Pneumatics() {


  }
  public void ClimberNeutral() {
    m_LdoubleSolenoid.set(DoubleSolenoid.Value.kOff);

  }
  public void ClimberAngled() {
    m_LdoubleSolenoid.set(DoubleSolenoid.Value.kForward);

  }
  public void ClimberStraight() {
    m_LdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  //  m_RdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
