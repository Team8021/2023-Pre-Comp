package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class InputDrive extends CommandBase {
  
  private final DrivetrainSubsystem m_subsystem;
  private final Supplier<Double> m_speedFunction, m_turnFunction;


  public InputDrive(DrivetrainSubsystem subsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    m_speedFunction = speedFunction;
    m_turnFunction = turnFunction;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

    System.out.println("InputDrive Command Started!");
  }

  @Override
  public void execute() {
    m_subsystem.setArcadeDrive(-m_speedFunction.get() * Constants.Speed_Limit.getDouble(.5), m_turnFunction.get() * Constants.Speed_Limit.getDouble(.5), false);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
