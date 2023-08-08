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