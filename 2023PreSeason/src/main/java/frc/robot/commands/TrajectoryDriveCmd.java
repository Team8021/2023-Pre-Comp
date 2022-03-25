// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryDriveCmd extends CommandBase {

  private final DrivetrainSubsystem m_subsystem;

  private TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(4));
  private Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    m_trajectoryConfig);

    private final RamseteController m_ramseteController = new RamseteController();
    private Timer m_timer;


  public TrajectoryDriveCmd(DrivetrainSubsystem subsystem) {
    System.out.println("Trajectory Drive Started!");
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }
  
  @Override
  public void initialize() {
    Constants.field.getObject("traj").setTrajectory(m_trajectory);
    m_timer = new Timer();
    m_timer.start();
    trajectoryDrive();
  }

  private void trajectoryDrive(){
    m_trajectoryConfig.setKinematics(m_subsystem.getKinematics());

  }

  @Override
  public void execute() {
    m_subsystem.feedDifferentialDrive();

    if(m_timer.get() < m_trajectory.getTotalTimeSeconds()){
      var desiredPose = m_trajectory.sample(m_timer.get());
      
      var refChassisSpeeds = m_ramseteController.calculate(m_subsystem.getPose(), desiredPose);

      m_subsystem.resetOdometry(m_trajectory.getInitialPose());

      m_subsystem.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    }else{
      m_subsystem.drive(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Trajectory Drive Cmd Stopped!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
