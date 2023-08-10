// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Pneumatics;
import frc.robot.Constants.ShootCommandConsants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shoot extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public shoot(Pneumatics tShirtSolenoid) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
    new InstantCommand(tShirtSolenoid::resetCooldownReady, tShirtSolenoid),
    new InstantCommand(tShirtSolenoid::OpenValve, tShirtSolenoid),
    // Wait Command keeps the valve open for a predetermined amount of time
    new WaitCommand(ShootCommandConsants.SECONDS),
    new InstantCommand(tShirtSolenoid::CloseValve, tShirtSolenoid),
    new WaitCommand(ShootCommandConsants.SECONDS),
    new InstantCommand(tShirtSolenoid::cooldownReady, tShirtSolenoid));
    
  }
}
