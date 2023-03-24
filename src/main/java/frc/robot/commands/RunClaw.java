// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunClaw extends CommandBase {
  /** Creates a new RunClaw. */
  public RunClaw() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double BUTTONFORINTAKE = RobotContainer.xbox.getLeftTriggerAxis();
    double BUTTONFORSPIT = RobotContainer.xbox.getRightTriggerAxis();
    double INTAKESPEED = BUTTONFORINTAKE - BUTTONFORSPIT;
    
    RobotContainer.m_claw.RunIntake(INTAKESPEED * .5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
