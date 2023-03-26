// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunArm extends CommandBase {
  /** Creates a new RunArm. */
  public RunArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Move arm up and down
    double xboxRightY = RobotContainer.xbox.getRightY();
    RobotContainer.m_arm.MoveArm(xboxRightY * .325);

    if (RobotContainer.xbox.getRightBumper()){
      RobotContainer.m_arm.ExtendArm(.4);
    } else if (RobotContainer.xbox.getLeftBumper()){
      RobotContainer.m_arm.ExtendArm(-.4);
    } else{
      RobotContainer.m_arm.ExtendArm(0);
    }
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
