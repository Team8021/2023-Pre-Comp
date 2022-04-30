package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HardwareConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  private final double kGearRatio = 10.7;
  private final double kWheelRadiusInches = 3.0;

  
  // Hardware
  private CANSparkMax m_leftMotor = new CANSparkMax(HardwareConstants.MOTOR_LEFT_ID, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(HardwareConstants.MOTOR_RIGHT_ID, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private Encoder m_leftAltEncoder = new Encoder(0, 1);
  private RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
  private Encoder m_rightAltEncoder = new Encoder(2, 3);
  private ADIS16448_IMU m_gyro = new ADIS16448_IMU();
  private SparkMaxPIDController m_rightMotorPID = m_rightMotor.getPIDController();
  private SparkMaxPIDController m_leftMotorPID = m_leftMotor.getPIDController();
  
  private DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  // Kinematics determine the voltage required to move the robot and stuff like that
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(.5);
  // Odometry determines where on the field you are
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());
  // Pose is where you are on the field
  Pose2d m_pose = new Pose2d();

  // Feedforward/feedback controllers.
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(.1191, 2., .5474);

  PIDController m_leftPIDController = new PIDController(.5, 0, 0);
  PIDController m_rightPIDController = new PIDController(.5, 0, 0);
  
  SlewRateLimiter forwardSlew = new SlewRateLimiter(Constants.FORWARD_SLEW_LIMIT);

  public DrivetrainSubsystem() {
    invertRightMotor();
    if(RobotState.isAutonomous()){
      m_leftMotor.setIdleMode(IdleMode.kBrake);
      m_rightMotor.setIdleMode(IdleMode.kBrake);
    }else{
      m_leftMotor.setIdleMode(IdleMode.kCoast);
      m_rightMotor.setIdleMode(IdleMode.kCoast);
    }
    m_leftAltEncoder.setDistancePerPulse((1./256.) * kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    m_rightAltEncoder.setDistancePerPulse((1./256.) * kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    // The conversion factor is in meters
    m_gyro.setYawAxis(IMUAxis.kZ);

    m_leftEncoder.setVelocityConversionFactor(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    m_rightEncoder.setVelocityConversionFactor(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    m_leftEncoder.setPositionConversionFactor(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    m_rightEncoder.setPositionConversionFactor(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    m_gyro.reset();
    m_odometry.resetPosition(new Pose2d(), getHeading());

    m_leftAltEncoder.reset();
    m_rightAltEncoder.reset();

    SmartDashboard.putData("Field", Constants.field);
  }

  //#region Setters
  public void calibrateGyro(){
    m_gyro.calibrate();
  }
  public void invertRightMotor() {
    m_rightMotor.setInverted(true);
  }
  public void setArcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    // double modifiedSpeed = (equals(xSpeed, 0, .1))? stopSlew.calculate(xSpeed) : forwardSlew.calculate(xSpeed); 
    
    m_differentialDrive.arcadeDrive(xSpeed * Constants.Speed_Limit.getDouble(.5), zRotation, squareInputs);
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
    m_rightMotor.set(rightVolts/12);
  }
  public void resetDrive()
  {
    m_gyro.reset();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(pose, getHeading());
    }
  public void feedDifferentialDrive(){
    m_differentialDrive.feed();
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    final double leftFeedForward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedForward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    // final double leftPID;
    // final double rightPID;

    // if(RobotBase.isReal()){
    //     leftPID = m_leftPIDController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    //     rightPID = m_rightPIDController.calculate(m_rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
    // }else{
    //     leftPID = m_leftPIDController.calculate(m_leftAltEncoder.getRate(), speeds.leftMetersPerSecond);
    //     rightPID = m_rightPIDController.calculate(m_rightAltEncoder.getRate(), speeds.rightMetersPerSecond);
    // }
    m_leftMotor.set((leftFeedForward)/12);
    m_rightMotor.set((rightFeedForward)/12);
  }
    /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
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
  public Pose2d getPose(){
    return m_pose;
  }
  public DifferentialDriveWheelSpeeds getSpeedsInMetersPerSecond(){
    return new DifferentialDriveWheelSpeeds(
    m_leftEncoder.getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
    m_rightEncoder.getVelocity() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }
  public SimpleMotorFeedforward getFeedForward(){
    return m_feedforward;
  }
  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics;
  }
  public PIDController getLeftPIDController(){
    return m_leftPIDController;
  }
  public PIDController getRightPIDController() {
    return m_rightPIDController;
  }
  //#endregion
  
  // Simulation Variables
  // Simulated Hardware
  
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftAltEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightAltEncoder);
  private ADIS16448_IMUSim m_gyroSim = new ADIS16448_IMUSim(m_gyro);

  // Simulated Drivetrain

  // DifferentialDrivetrainSim m_drivetrainSim = new DifferentialDrivetrainSim(  
  //   LinearSystemId.identifyDrivetrainSystem(Constants.KvLinear, Constants.KaLinear, Constants.KvAngular, Constants.KaAngular),
  //   DCMotor.getNEO(1),       // 1 NEO motors on each side of the drivetrain.
  //   7.29,                    // 7.29:1 gearing reduction.
  //   7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  //   Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  //   // The standard deviations for measurement noise:
  //   // x and y:          0.001 m
  //   // heading:          0.001 rad
  //   // l and r velocity: 0.1   m/s
  //   // l and r position: 0.005 m
  //   VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    DifferentialDrivetrainSim m_drivetrainSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(1),       // 2 NEO motors on each side of the drivetrain.
      10.7,                    // 7.29:1 gearing reduction.
      7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
      60.0,                    // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.5461,                  // The track width in meters

      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  @Override
  public void periodic() {
    if(RobotBase.isReal()){
      m_pose = m_odometry.update(getHeading(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    }else{
      m_pose = m_odometry.update(getHeading(), m_leftAltEncoder.getDistance(), m_rightAltEncoder.getDistance());
    }
    Constants.field.setRobotPose(m_pose);
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
  private boolean equals(double a, double b, double epsilon){
    if(a==b) return true;
    return Math.abs(a - b) < epsilon;
  }
}