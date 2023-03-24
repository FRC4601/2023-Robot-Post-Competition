// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.ScoreMidCubeAndDriveOut;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoldArmMidCube extends CommandBase {
  /** Creates a new HoldArmMidCube. */
  public HoldArmMidCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ArmEncoderPos = RobotContainer.m_arm.GetLeftArmPosition();
    if (ArmEncoderPos <= 9.3){
      RobotContainer.m_arm.MoveArm(-.16);
    } else{
      RobotContainer.m_arm.MoveArm(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.MoveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_arm.GetLeftArmPosition() >= 10;
  }
}
