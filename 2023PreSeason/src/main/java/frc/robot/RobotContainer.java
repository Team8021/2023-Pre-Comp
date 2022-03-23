package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.InputDrive;
import frc.robot.subsystems.DrivetrainSubsystem;


public class RobotContainer {


  XboxController m_driveController = new XboxController(0);


  private final DrivetrainSubsystem m_dDrivetrainSubsystem = new DrivetrainSubsystem();

  public RobotContainer() {


    m_dDrivetrainSubsystem.setDefaultCommand(new InputDrive(m_dDrivetrainSubsystem, () -> m_driveController.getRawAxis(ControllerConstants.LEFT_Y), () -> m_driveController.getRawAxis(4)));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
