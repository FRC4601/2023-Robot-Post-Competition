// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private final VictorSPX clawLeftMotor = new VictorSPX(5);
  private final VictorSPX clawRightMotor = new VictorSPX(6);

  private final DoubleSolenoid ClawSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  /** Creates a new Claw. */
  public Claw() {
    clawRightMotor.follow(clawLeftMotor);
    clawRightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunIntake(double speed){
    clawLeftMotor.set(ControlMode.PercentOutput, speed);
  } 
  public void StopIntake(){
    clawLeftMotor.set(ControlMode.PercentOutput, 0);
  }

  public void CloseClaw(){
    ClawSol.set(Value.kForward);
  }
  public void OpenClaw(){
    ClawSol.set(Value.kReverse);
  }
}
