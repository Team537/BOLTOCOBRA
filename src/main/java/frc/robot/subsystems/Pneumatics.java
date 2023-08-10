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
  
  private final Timer sincePressedTimer = new Timer();
  public boolean safety = false;
  public boolean cooldownReady = true;

  PneumaticHub pneumaticHub = new PneumaticHub(PneumaticConstants.MODULE_NUMBER);  
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


  
  public boolean canShoot(int leftTriggerState, int rightTriggerState){
     
    // System.out.println(m_driverController.getLeftTriggerAxis() + " " + m_driverController.getRightTriggerAxis());
    if (leftTriggerState != 0){ //If left trigger is being pressed
     
      // Restarts/starts time when the left trigger pressed down
      if(leftTriggerState == 1){
        sincePressedTimer.reset();
        sincePressedTimer.start();
      }

     

      // Checks if the right trigger just got pressed
    
        if (sincePressedTimer.hasElapsed(PneumaticConstants.SAFTEY_DELAY) ){
          safety = true;
          if(rightTriggerState == 1){
             System.out.println("FIRE THE MAIN CANNONS");
          sincePressedTimer.reset();
          safety = false;
          
          return true;
        
        } // Checks if the <SAFTEY_DELAY> has passed
           // Resets the timer if you shoot, so you have to wait again before firing
           
        }
        
        
        // Resets the timer if you press the right trigger
        // So you have to wait again before firing (Requiries a reset timer before the return true to work properly)
         
      
    } else {

      safety = false;

    }
    
    return false;
  }

  //non constant variables
  public int iSolenoid = 0;
  
  public boolean reset = false;
  public boolean arrayDone = false;

  public Pneumatics() {
    

  }

  public void Neutral() {
    // Neutralized valve # iSolenoid
    SolenoidList[iSolenoid].set(false);
    System.out.println("Barrel Neutral: " + iSolenoid);
  }

  public void OpenValve() {
    if (arrayDone == false) {

       
      // Opens valve # iSolenoid
      SolenoidList[iSolenoid].set(true);
      System.out.println("Barrel Opened: " + iSolenoid);
    }
  }

  public void CloseValve () {
    if (arrayDone == false) {
      
      // Closes valve # iSolenoid
      SolenoidList[iSolenoid].set(false);
      System.out.println("Barrel Closed: " + iSolenoid);

      // Iterates through the soliniods in Solenoid List
      iSolenoid += 1;
      System.out.println("Solendoid incremented to :" + iSolenoid);

      // Upon hitting the end of the list, set the iterator back to 0

      //autoloop removed
      if (iSolenoid >= SolenoidList.length) {
        arrayDone = true;
        // System.out.println("Solenoid List Reset");
      }
    }
  }

  public void ResetShootArray() {
    
   
      iSolenoid = 0;
      System.out.println("Solenoid iterator set to 0");
      
      arrayDone = false;
      //resets the bool so you dont accidently reset array again
      reset = false;
      
    

  }

  public void cooldownReady() {
   cooldownReady = true;
  }

  public void resetCooldownReady() {
    cooldownReady = false;
   }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(" Active Solenoid Cycle Num", iSolenoid);
    SmartDashboard.putBoolean("Cooldown Ready", cooldownReady);
    SmartDashboard.putBoolean("Safety Ready", safety);
    SmartDashboard.putBoolean("Solenoid Array Done", arrayDone);
   
    

    

  }


  
}
