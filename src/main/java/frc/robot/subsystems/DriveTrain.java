// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  //Falcon motor controllers
  private final PWMVictorSPX frontleftMotor = new PWMVictorSPX(1);
  private final PWMVictorSPX backleftMotor = new PWMVictorSPX(2);
  private final PWMVictorSPX frontrightMotor = new PWMVictorSPX(3);
  private final PWMVictorSPX backrightMotor = new PWMVictorSPX(4);

  //Differentialdrive groups
  private final MotorControllerGroup m_leftDriveGroup = new MotorControllerGroup(frontleftMotor, backleftMotor);
  private final MotorControllerGroup m_rightDriveGroup = new MotorControllerGroup(frontrightMotor, backrightMotor);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);

  //Odometry
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  //Compressor/Solenoids Inits
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  


  /** Creates a new DriveTrain. */
  public DriveTrain(){
    m_leftDriveGroup.setInverted(true); //Invert one side drive motors
    ResetGyro();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    compressor.enableDigital();

    SmartDashboard.putNumber("Gyro Roll (X)", GetGyroXAngle());
    SmartDashboard.putNumber("Gyro Pitch (Y)", GetGyroYAngle());
    SmartDashboard.putNumber("Gyro Yaw (Angle)", GetGyroYaw());
  }


  public void ArcadeDrive(double lspeed, double rspeed){   //Our main ArcadeDrive command. 
    m_drive.tankDrive(lspeed, rspeed, false);
  } 
  public void ArcadeDrive(double speed, double rotation, boolean isSquaredInputs){   //Secondary ArcadeDrive command. Has additional bool for squared inputs to increase controlability at low speeds. 
    m_drive.arcadeDrive(speed, rotation, isSquaredInputs);
  }

  public void ResetGyro(){
    gyro.reset();
  }

  public double GetGyroYaw(){
    return gyro.getAngle();
  }
  public double GetGyroXAngle(){
    return gyro.getXComplementaryAngle();
  }
  public double GetGyroYAngle(){
    return gyro.getYComplementaryAngle();
  }

}
