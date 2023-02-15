// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.cscore.MjpegServer;
//import edu.wpi.first.wpilibj.smartdashboard.*;

public class camera extends SubsystemBase {
  /** Creates a new Camera. */
  public camera() {
  UsbCamera cam =  new UsbCamera("Usb Camera 0", 0);

  MjpegServer mjep = new MjpegServer("server_1", 1181);
   mjep.setSource(cam);
  CameraServer.startAutomaticCapture(0);
  

  }
}


  


