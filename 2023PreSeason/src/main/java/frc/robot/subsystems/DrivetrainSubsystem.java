package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

  private final double kGearRatio = 7.29;
  private final double kWheelRadiusInches = 3.0;

  
  // Hardware
  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.MOTOR_LEFT_ID, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private Encoder m_leftAltEncoder = new Encoder(0, 1);
  private RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
  private Encoder m_rightAltEncoder = new Encoder(2, 3);
  private ADIS16448_IMU m_gyro = new ADIS16448_IMU();
  
  private DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  // Kinematics determine the voltage required to move the robot and stuff like that
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  // Odometry determines where on the field you are
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());
  // Pose is where you are on the field
  Pose2d m_pose = new Pose2d();

  // Feedforward/feedback controllers. THESE ARE NOT COMPLETED THE CONSTANTS ARE NOT SPECIFIC TO OUR ROBOT PLEASE CHANGE FOR THE LOVE OF GOD
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  PIDController m_leftPIDController = new PIDController(2.95, 0, 0);
  PIDController m_rightPIDController = new PIDController(2.95, 0, 0);
  
  public DrivetrainSubsystem() {
    // The conversion factor is in meters

    m_leftAltEncoder.setDistancePerPulse(kGearRatio * 2 * Math.PI / 42);
    m_rightAltEncoder.setDistancePerPulse(kGearRatio * 2 * Math.PI / 42);

    m_leftEncoder.setPositionConversionFactor(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    m_rightEncoder.setPositionConversionFactor(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    m_gyro.reset();

    SmartDashboard.putData("Field", m_field);
  }

  //#region Setters
  public void invertRightMotor() {
    m_rightMotor.setInverted(true);
  }
  public void setArcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_differentialDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }
  public void setCurvatureDrive(double xSpeed, double zRotation) {
    m_differentialDrive.curvatureDrive(xSpeed, zRotation, true);
  }
  public void setTankDrive(double leftSpeed, double rightSpeed) {
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
  public void stopDrive() {
    m_differentialDrive.stopMotor();
  }
  public void setOutputVolts(double leftVolts, double rightVolts){
    m_leftMotor.set(leftVolts/12);
    m_rightMotor.set(leftVolts/12);
  }
  public void resetDrive()
  {
    m_gyro.reset();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(new Pose2d(), getHeading());
  }
  //#endregion
  //#region Getters

  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }
  public double getRightEncoderVelocity() {
    return m_rightEncoder.getVelocity();
  }
  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }
  public double getRightEncoderPosition() {
    return m_rightEncoder.getPosition();
  }
  public double getGyroAngle(){
    return m_gyro.getAngle();
  }
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }
  // public DifferentialDriveWheelSpeeds getSpeedsInMetersPerSecond(){
  //   return new DifferentialDriveWheelSpeeds(
  //   m_leftEncoder.getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
  //   m_rightEncoder.getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
  //   );
  // }
  public SimpleMotorFeedforward getFeedForward(){
    return m_feedforward;
  }
  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics;
  }
  public PIDController getLeftPidController(){
    return m_leftPIDController;
  }
  public PIDController getRightPIDController() {
    return m_rightPIDController;
  }

  // Simulation Variables
  // Simulated Hardware
  
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftAltEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightAltEncoder);
  private ADIS16448_IMUSim m_gyroSim = new ADIS16448_IMUSim(m_gyro);

  // Simulated Drivetrain

  DifferentialDrivetrainSim m_drivetrainSim = new DifferentialDrivetrainSim(  
    LinearSystemId.identifyDrivetrainSystem(0.3, 1.96, 0.06, .06),
    DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
    7.29,                    // 7.29:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    Units.inchesToMeters(3), // The robot uses 3" radius wheels.
    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    // Simulated field
    private Field2d m_field = new Field2d();

  //#endregion
  @Override
  public void periodic() {
    m_pose = m_odometry.update(getHeading(), m_leftAltEncoder.getDistance(), m_rightAltEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }
  public void simulationPeriodic(){
    m_drivetrainSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
                              m_rightMotor.get() * RobotController.getInputVoltage());
    m_drivetrainSim.update(0.02);
    // updating the simulated hardware
    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setGyroAngleZ(-m_drivetrainSim.getHeading().getDegrees());
  }
}