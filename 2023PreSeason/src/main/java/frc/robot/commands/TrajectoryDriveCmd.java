// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryDriveCmd extends CommandBase {

  private final DrivetrainSubsystem m_subsystem;

  private boolean isFinished = false;

  private TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));

  String trajectoryJSON = "paths/Test.wpilib.json";
  Trajectory m_trajectory = new Trajectory();

    private final RamseteController m_ramseteController = new RamseteController();
    private Timer m_timer;


  public TrajectoryDriveCmd(DrivetrainSubsystem subsystem) {
    System.out.println("Trajectory Drive Started!");
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }
  
  @Override
  public void initialize() {

    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch (IOException ex){
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    Constants.field.getObject("traj").setTrajectory(m_trajectory);
    m_timer = new Timer();
    m_timer.start();
    m_trajectoryConfig.setKinematics(m_subsystem.getKinematics());
    m_subsystem.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void execute() {
    m_subsystem.feedDifferentialDrive();

    if(m_timer.get() < m_trajectory.getTotalTimeSeconds()){
      Trajectory.State desiredPose = m_trajectory.sample(m_timer.get());
      
      ChassisSpeeds refChassisSpeeds = m_ramseteController.calculate(m_subsystem.getPose(), desiredPose);

      m_subsystem.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    }else{
      m_subsystem.setTankDrive(0, 0);
      isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setTankDrive(0, 0);
    System.out.println("Trajectory Drive Cmd Stopped!");
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
