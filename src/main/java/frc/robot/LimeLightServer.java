// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimeLightServer {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //private NetworkTable table2 = NetworkTableInstance.getDefault().getTable("CameraPublisher").getSubTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry tid = table.getEntry("tid");
  //private NetworkTableEntry available = table2.getEntry("description");

  public double getX() {
      return tx.getDouble(0.0);
  }

  public double getY() {
      return ty.getDouble(0.0);
  }

  public double getID() {
      return tid.getDouble(-1.0);
  }

  public boolean availableTarget() {
      if (tv.getDouble(0.0) > 0.0)
          return true;
      else
          return false;
  }
}
