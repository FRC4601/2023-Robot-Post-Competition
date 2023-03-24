// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.BalanceOnChargeStation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveToChargeStation extends CommandBase {
  double SteerWithCorrection = 0;

  /** Creates a new DriveToChargeStation. */
  public DriveToChargeStation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RobotYaw = RobotContainer.m_drivetrain.GetGyroYaw();

    if (RobotYaw > 2.5){
      SteerWithCorrection = .085;
    } else if (RobotYaw < 2.5){
      SteerWithCorrection = -.085;
    } else{
      SteerWithCorrection = 0;
    }    

    RobotContainer.m_drivetrain.ArcadeDrive(-.25, SteerWithCorrection); // negative angle turns right
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.xbox.getYButton();
  }
}
