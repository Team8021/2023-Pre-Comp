// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryDriveCmd extends CommandBase {

  private final DrivetrainSubsystem m_subsystem;

  private boolean isFinished = false;

  Trajectory m_trajectory = new Trajectory();

    private final RamseteController m_ramseteController = new RamseteController();
    private Timer m_timer;
    private boolean m_resetPose;


  public TrajectoryDriveCmd(DrivetrainSubsystem subsystem, Trajectory trajectory, boolean resetPose) {
    m_resetPose = resetPose;
    System.out.println("Trajectory Drive Started!");
    m_trajectory = trajectory;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }
  
  @Override
  public void initialize() {
    m_subsystem.calibrateGyro();
    if(m_resetPose){
      m_subsystem.resetOdometry(m_trajectory.getInitialPose());
    }
    Constants.field.getObject("traj").setTrajectory(m_trajectory);
    m_timer = new Timer();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_subsystem.feedDifferentialDrive();

    if(m_timer.get() < m_trajectory.getTotalTimeSeconds()){
      Trajectory.State desiredPose = m_trajectory.sample(m_timer.get());
      
      ChassisSpeeds refChassisSpeeds = m_ramseteController.calculate(m_subsystem.getPose(), desiredPose);

      m_subsystem.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    }else{
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
