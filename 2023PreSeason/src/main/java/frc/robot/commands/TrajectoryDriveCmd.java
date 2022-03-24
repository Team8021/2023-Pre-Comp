// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryDriveCmd extends CommandBase {

  private final DrivetrainSubsystem m_subsystem;



  private TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(4));
  private Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(

    // Start at the origin facing the +X direction

    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    m_trajectoryConfig);


  public TrajectoryDriveCmd(DrivetrainSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }
  
  @Override
  public void initialize() {

  }

  public Command trajectoryDrive(){
    m_trajectoryConfig.setKinematics(m_subsystem.getKinematics());
    RamseteCommand command = new RamseteCommand(
            m_trajectory,
            m_subsystem::getPose,
            new RamseteController(2, .7),
            m_subsystem.getFeedForward(),
            m_subsystem.getKinematics(),
            m_subsystem::getSpeedsInMetersPerSecond,
            m_subsystem.getLeftPIDController(),
            m_subsystem.getRightPIDController(),
            m_subsystem::setOutputVolts,
            m_subsystem
        );
    m_subsystem.resetOdometry(m_trajectory.getInitialPose());

    return command.andThen(() -> m_subsystem.setOutputVolts(0, 0));
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
