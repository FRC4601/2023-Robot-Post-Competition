// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final WPI_TalonFX leftFalcon = new WPI_TalonFX(0);
  private final WPI_TalonFX rightFalcon = new WPI_TalonFX(1);

  private final PIDController m_armPID = new PIDController(.5, 0, 0);
  private final DigitalInput armLimitSwitch = new DigitalInput(0);

  private final CANSparkMax leftWinch = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightWinch = new CANSparkMax(3, MotorType.kBrushless);

  // private final Solenoid sol1 = new Solenoid(2, PneumaticsModuleType.CTREPCM, 0);

  /** Creates a new Arm. */
  public Arm() {
    //arm motors
    leftFalcon.configFactoryDefault();
    rightFalcon.configFactoryDefault();    
    
    rightFalcon.follow(leftFalcon);
    rightFalcon.setInverted(true);

    leftFalcon.setNeutralMode(NeutralMode.Brake);
    rightFalcon.setNeutralMode(NeutralMode.Brake);

    leftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    //winch motors
    rightWinch.follow(leftWinch, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Position", GetLeftArmPosition());
    SmartDashboard.putNumber("Right Arm Encoder Position", GetRightArmPosition());
  }

  public void MoveArm(double speed){
    leftFalcon.set(ControlMode.PercentOutput, speed);
  }
  public void SetArmToPoint(double setpoint){
    leftFalcon.set(ControlMode.PercentOutput, m_armPID.calculate(GetLeftArmPosition(), setpoint));
  }
  public void ExtendArm(double speed){
    leftWinch.set(speed);
  }


  public double GetLeftArmPosition(){
    return -leftFalcon.getSelectedSensorPosition() / 2048;
  }
  public double GetRightArmPosition(){
    return -rightFalcon.getSelectedSensorPosition() / 2048;
  }
  public void ResetArmEncoders(){
    leftFalcon.setSelectedSensorPosition(0);
    rightFalcon.setSelectedSensorPosition(0);
  }

  public boolean GetLimitSwitchPressed(){
    return (armLimitSwitch.get());
  }

}
