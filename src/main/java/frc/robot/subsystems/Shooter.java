//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PneumaticConstants;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends SubsystemBase { 
  private final WPI_TalonFX m_Intake = new WPI_TalonFX(Constants.IntakeConstants.kIntake);
  long start_time = -1;
  Timer timer_since_pressed = new Timer();

  public boolean can_shoot(int left_trigger_state, int right_trigger_state){
    System.out.println(left_trigger_state + " " + right_trigger_state);
    // System.out.println(m_driverController.getLeftTriggerAxis() + " " + m_driverController.getRightTriggerAxis());
    if (left_trigger_state != 0){ //If left trigger is being pressed
      if(left_trigger_state == 1){ // Set start time only on the press down
        // start_time = System.currentTimeMillis();
        Timer timer_since_pressed = new Timer();
      }

      // long current_time = System.currentTimeMillis();
      // System.out.println(start_time + " " + current_time);
      // ((current_time - start_time) >= PneumaticConstants.SAFTEY_DELAY)
      if (right_trigger_state != 0 && timer_since_pressed.hasElapsed(PneumaticConstants.SAFTEY_DELAY)){ //If right trigger is pressed down past half way
        System.out.println("FIRE THE MAIN CANNONS");
        // start_time = System.currentTimeMillis();  // Resets the cooldown, so you have to wait another <SAFTEY_DELAY> before shooting
        timer_since_pressed = new Timer();
        return true;
      }
    }
    else{
      // reset start time
      start_time = -1;
    }
    return false;
  }

  /** Creates a new Intake. */
  public Shooter() {
    
  //m_Intake.configFactoryDefault(Constants.kTimeoutMs);
  m_Intake.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void intakeIn() {
    m_Intake.set(0.8);
  }
  public void intakeOff() {
    m_Intake.set(0.0);
  }
  public void intakeOut() {
    m_Intake.set(-0.5);
  }


}