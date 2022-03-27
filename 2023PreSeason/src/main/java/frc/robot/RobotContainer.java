package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.InputDrive;
import frc.robot.commands.TrajectoryDriveCmd;
import frc.robot.subsystems.DrivetrainSubsystem;


public class RobotContainer {

  XboxController m_driveController = new XboxController(0);
  private static ShuffleboardTab tab = Shuffleboard.getTab("Constants");

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  public RobotContainer() {
    tab.add(Constants.AutoRoutine);
    Constants.AutoRoutine.addOption("Auto Routine # 1",
    new SequentialCommandGroup(
      new TrajectoryDriveCmd(m_drivetrainSubsystem, GenerateTrajectoryFromJSON("paths/Ball1.wpilib.json"), true),
      new TrajectoryDriveCmd(m_drivetrainSubsystem, GenerateTrajectoryFromJSON("paths/Ball2.wpilib.json"), false)
      )
    );
    Constants.AutoRoutine.addOption("Auto Routine # 2",
    new SequentialCommandGroup(
      new TrajectoryDriveCmd(m_drivetrainSubsystem, GenerateTrajectoryFromJSON("paths/Ball3.wpilib.json"), true),
      new TrajectoryDriveCmd(m_drivetrainSubsystem, GenerateTrajectoryFromJSON("paths/Ball4.wpilib.json"), false),
      new TrajectoryDriveCmd(m_drivetrainSubsystem, GenerateTrajectoryFromJSON("paths/Ball5.wpilib.json"), false)
    )
  );

    m_drivetrainSubsystem.setDefaultCommand(new InputDrive(m_drivetrainSubsystem, () -> m_driveController.getRawAxis(ControllerConstants.LEFT_Y), () -> m_driveController.getRawAxis(4)));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Constants.AutoRoutine.getSelected();

  }
  private Trajectory GenerateTrajectoryFromJSON(String trajectoryJSON){
    try{
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        return trajectory;
      }catch (IOException ex){
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        return null;
      }
  }
}
