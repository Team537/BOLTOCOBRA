// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shoot extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public shoot(Pneumatics m_tshirtsolenoid) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new StartEndCommand(m_tshirtsolenoid::OpenValve,m_tshirtsolenoid::OpenValve, m_tshirtsolenoid),
    new WaitCommand(1),
    new StartEndCommand(m_tshirtsolenoid::CloseValve,m_tshirtsolenoid::CloseValve, m_tshirtsolenoid));

  }
}
