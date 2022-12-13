// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimeLight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LLFollowCmd extends CommandBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  PIDController rotationPID = new PIDController(.02, 0, 0);
  PIDController drivePID = new PIDController(.001, 0, 0);
  DrivetrainSubsystem m_subsystem;

  /** Creates a new LLFollowCmd. */
  public LLFollowCmd(DrivetrainSubsystem m_subsystem) {
    this.m_subsystem = m_subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double a1 = LimeLight.MOUNT_ANGLE_DEG * (3.14159 / 180.0);
    double a2  = y * (3.14159 / 180.0);
    double angle = a1 + a2;

    
    double dist = (30.0 - LimeLight.LENS_HEIGHT_INC)/Math.tan(angle);
    double inputDis;
    if(dist > -30)
    {
      inputDis = 100;
    }
    else
    {
      inputDis = dist;
    }

    SmartDashboard.putNumber("Dist", dist);
    SmartDashboard.putNumber("DrivePID output", drivePID.calculate(inputDis));
    // SmartDashboard.putNumber("Real Dist", realDist);

    // m_subsystem.rotate();
    m_subsystem.setArcadeDrive(0, rotationPID.calculate(-x), false);
    // drivePID.calculate(inputDis * 2)
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);


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
