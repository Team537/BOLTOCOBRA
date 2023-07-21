// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PneumaticConstants;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.chrono.IsoChronology;
import java.util.List;

import com.ctre.phoenixpro.signals.IsPROLicensedValue;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  PneumaticHub m_PneumaticHub = new PneumaticHub(PneumaticConstants.MODULE_NUMBER);  
  // private static int RforwardChannel =0;
  // private static int RreverseChannel =1;
  /** Creates a new Pneumatics. */
  public final Solenoid SolenoidList[] = {
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_1_CHANNEL),
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_2_CHANNEL),
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_3_CHANNEL),
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_4_CHANNEL),
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_5_CHANNEL),
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_6_CHANNEL),
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_7_CHANNEL),
    new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticConstants.T_SHIRT_SOLENOID_8_CHANNEL),
  };
  private boolean Tube0;
  private boolean Tube1;
  private boolean Tube2;
  private boolean Tube3;
  private boolean Tube4;
  private boolean Tube5;
  private boolean Tube6;
  private boolean Tube7;
  public final boolean TubeList[] = {
    Tube0 = true,
    Tube1 = true,
    Tube2 = true,
    Tube3 = true,
    Tube4 = true,
    Tube5 = true,
    Tube6 = true,
    Tube7 = true
  };

  public int iSolenoid = 0;
  public boolean safety = false;
 // DoubleSolenoid m_RdoubleSolenoid = m_PneumaticHub.makeDoubleSolenoid(RforwardChannel,RreverseChannel);

  public Pneumatics() {


  }

  public void Neutral() {
    // Neutralized valve # iSolenoid
    SolenoidList[iSolenoid].set(false);
    System.out.println("Barrel Neutral: " + iSolenoid);
  }

  public void OpenValve() {
    GetTubeBool();
    if (TubeList[iSolenoid] == true) {
      SmartDashboard.putBoolean("Active Tube" + iSolenoid, true);

      // Opens valve # iSolenoid
      SolenoidList[iSolenoid].set(true);
    }
      System.out.println("Barrel Opened: " + iSolenoid);
  }

  public void CloseValve () {
    if(TubeList[iSolenoid] == true) {
      SmartDashboard.putBoolean("Active Tube " + iSolenoid, false);

      // Closes valve # iSolenoid
      SolenoidList[iSolenoid].set(false);
    }

    System.out.println("Barrel Closed: " + iSolenoid);

    // Iterates through the soliniods in Solenoid List
    iSolenoid += 1;
    System.out.println("Solendoid incremented to :" + iSolenoid);

    // Upon hitting the end of the list, set the iterator back to 0
    if (iSolenoid >= SolenoidList.length) {
      iSolenoid = 0;

      System.out.println("Solenoid List Reset");
    }

    //  m_RdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.setDefaultBoolean("Tube 0 Status", TubeList[0]);
    SmartDashboard.setDefaultBoolean("Tube 1 Status", TubeList[1]);
    SmartDashboard.setDefaultBoolean("Tube 2 Status", TubeList[2]);
    SmartDashboard.setDefaultBoolean("Tube 3 Status", TubeList[3]);
    SmartDashboard.setDefaultBoolean("Tube 4 Status", TubeList[4]);
    SmartDashboard.setDefaultBoolean("Tube 5 Status", TubeList[5]);
    SmartDashboard.setDefaultBoolean("Tube 6 Status", TubeList[6]);
    SmartDashboard.setDefaultBoolean("Tube 7 Status", TubeList[7]);

    SmartDashboard.putBoolean("Active Tube 0", false);
    SmartDashboard.putBoolean("Active Tube 1", false);
    SmartDashboard.putBoolean("Active Tube 2", false);
    SmartDashboard.putBoolean("Active Tube 3", false);
    SmartDashboard.putBoolean("Active Tube 4", false);
    SmartDashboard.putBoolean("Active Tube 5", false);
    SmartDashboard.putBoolean("Active Tube 6", false);
    SmartDashboard.putBoolean("Active Tube 7", false);

    

  }

  public void GetTubeBool() {
    TubeList[iSolenoid] = SmartDashboard.getBoolean("Tube " + iSolenoid + " Status", TubeList[iSolenoid]);
  //   TubeList[0] = SmartDashboard.getBoolean("Tube 0 Status", Tube0);
  //   TubeList[1] = SmartDashboard.getBoolean("Tube 1 Status", Tube1);
  //   TubeList[2] = SmartDashboard.getBoolean("Tube 2 Status", Tube2);
  //   TubeList[3] = SmartDashboard.getBoolean("Tube 3 Status", Tube3);
  //   TubeList[4] = SmartDashboard.getBoolean("Tube 4 Status", Tube4);
  //   TubeList[5] = SmartDashboard.getBoolean("Tube 5 Status", Tube5);
  //   TubeList[6] = SmartDashboard.getBoolean("Tube 6 Status", Tube6);
  //   TubeList[7] = SmartDashboard.getBoolean("Tube 7 Status", Tube7);
  }
  
}
