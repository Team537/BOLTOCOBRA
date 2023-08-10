// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PneumaticConstants;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.chrono.IsoChronology;
import java.util.List;

import com.ctre.phoenixpro.signals.IsPROLicensedValue;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  long start_time = -1;
  Timer timer_since_pressed = new Timer();

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


  
  public boolean can_shoot(int left_trigger_state, int right_trigger_state){
    
    // System.out.println(m_driverController.getLeftTriggerAxis() + " " + m_driverController.getRightTriggerAxis());
    if (left_trigger_state != 0){ //If left trigger is being pressed

      // Restarts/starts time when the left trigger pressed down
      if(left_trigger_state == 1){
        timer_since_pressed.reset();
        timer_since_pressed.start();
      }

      // Checks if the right trigger just got pressed
      if(right_trigger_state == 1){
        if (timer_since_pressed.hasElapsed(PneumaticConstants.SAFTEY_DELAY)){ // Checks if the <SAFTEY_DELAY> has passed
          System.out.println("FIRE THE MAIN CANNONS");
          timer_since_pressed.reset(); // Resets the timer if you shoot, so you have to wait again before firing
          return true;
        }
        // Resets the timer if you press the right trigger
        // So you have to wait again before firing (Requiries a reset timer before the return true to work properly)
        timer_since_pressed.reset();  
      }
    }
    return false;
  }

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

    SmartDashboard.putBoolean("Active Tube" + iSolenoid, true);
    // Opens valve # iSolenoid
    SolenoidList[iSolenoid].set(true);
    System.out.println("Barrel Opened: " + iSolenoid);
  }

  public void CloseValve () {
   
    SmartDashboard.putBoolean("Active Tube " + iSolenoid, false);
    // Closes valve # iSolenoid
    SolenoidList[iSolenoid].set(false);
    System.out.println("Barrel Closed: " + iSolenoid);

    // Iterates through the soliniods in Solenoid List
    iSolenoid += 1;
    System.out.println("Solendoid incremented to :" + iSolenoid);

    // Upon hitting the end of the list, set the iterator back to 0

    //autoloop removed
    /**if (iSolenoid >= SolenoidList.length) {
      iSolenoid = 0;

      System.out.println("Solenoid List Reset");
    }**/

    //  m_RdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Active Tube 0", false);
    SmartDashboard.putBoolean("Active Tube 1", false);
    SmartDashboard.putBoolean("Active Tube 2", false);
    SmartDashboard.putBoolean("Active Tube 3", false);
    SmartDashboard.putBoolean("Active Tube 4", false);
    SmartDashboard.putBoolean("Active Tube 5", false);
    SmartDashboard.putBoolean("Active Tube 6", false);
    SmartDashboard.putBoolean("Active Tube 7", false);

    

  }


  
}
