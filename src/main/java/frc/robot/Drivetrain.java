package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second.
  public static final double kMaxAngularSpeed = Math.PI;// 1/2 rotation per second.
  private static final double kTrackWidth = 0.381 * 2;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = -4096;

  private final PWMSparkMax leftLeader = new PWMSparkMax(1);
  private final PWMSparkMax leftFollower = new PWMSparkMax(2);
  private final PWMSparkMax rightLeader = new PWMSparkMax(3);
  private final PWMSparkMax rightFollower = new PWMSparkMax(4);

  private final MotorControllerGroup leftGroup =
      new MotorControllerGroup(leftLeader, leftFollower);
  private final MotorControllerGroup rightGroup =
      new MotorControllerGroup(rightLeader, rightFollower);

  private final Encoder leftEncoder = new Encoder(0, 1);
  private final Encoder rightEncoder = new Encoder(2, 3);

  private final PIDController leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController rightPIDController = new PIDController(8.5, 0, 0);

  private final AnalogGyro gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(
          gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

  // Gains are for example purposes only - you can have fun and modify the values to see how things change
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

  // Simulation classes help us simulate our robot
  private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
  private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  private final Field2d fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim drivetrainSimulator =
      new DifferentialDrivetrainSim(
          drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

  /*
   * TODO
   * Set up a right encoder. Look at what functions the left encoder uses and modify as needed
   */
  public Drivetrain() {  /** Subsystem constructor. */
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    leftEncoder.reset();


    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward.
    rightGroup.setInverted(true);
    SmartDashboard.putData("Field", fieldSim);
  }

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
  /*
   * TODO
   * Set the speeds of the right side of the drive base
   * Have fun!
   */
    var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    double leftOutput = leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    leftGroup.setVoltage(leftOutput + leftFeedforward);
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis
   * @param rot the rotation
   */
  public void drive(double xSpeed, double rot) {
    setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    drivetrainSimulator.setPose(pose);
    odometry.resetPosition(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    drivetrainSimulator.setInputs(
        leftGroup.get() * RobotController.getInputVoltage(),
        rightGroup.get() * RobotController.getInputVoltage());
    drivetrainSimulator.update(0.02);
  /*
   * TODO
   * Set up a right encoder. Look at what functions the left encoder uses and modify as needed
   */
    leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
    gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
    updateOdometry();
    fieldSim.setRobotPose(odometry.getPoseMeters());
  }
}
