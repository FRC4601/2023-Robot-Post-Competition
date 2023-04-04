// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Vision extends SubsystemBase {
  //USB Camera(s)
  private final CvSink m_cvSink;

  //public Vision() {
  //  UsbCamera camera = new UsbCamera("Camera", 0);
  //  camera.setBrightness(3);
  //  CameraServer.startAutomaticCapture(camera);
  //  m_cvSink = CameraServer.getVideo();
    
  public Vision() {
    CameraServer.startAutomaticCapture(); //start USB camera on RoboRIO
    m_cvSink = CameraServer.getVideo();

  }



  @Override
  public void periodic() { // This method will be called once per scheduler run
   
  }
}
