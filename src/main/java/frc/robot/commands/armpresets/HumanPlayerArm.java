// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;
import frc.robot.subsystems.Arm;

public class HumanPlayerArm extends CommandBase {
  /** Creates a new HumanPlayerArm. */
  public HumanPlayerArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ArmPos = RobotContainer.m_arm.GetLeftArmPosition();
    if (ArmPos <= 6){
      RobotContainer.m_arm.MoveArm(-.25);
    } else if (ArmPos >= 6 && ArmPos <= 8){
      RobotContainer.m_arm.MoveArm(-.2);
    } else if (ArmPos > 8){
      RobotContainer.m_arm.MoveArm(-.15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.StopArm();
    RobotContainer.m_arm.setDefaultCommand(new RunArm());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.xbox.getRightY() > .1 || RobotContainer.xbox.getRightY() < -.1 || RobotContainer.xbox.getPOV() == 180;
  }
}
