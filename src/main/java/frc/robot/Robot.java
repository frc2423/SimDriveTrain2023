package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import java.util.List;


public class Robot extends TimedRobot {

  private final Drivetrain m_drive = new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;

  @Override
  public void robotInit() {
    /* TODO
     * you can create a trajectory. 
     * Set the trajectory variable and use TrajectoryGenerator in order to make the trajectory
     * It has three parameters, an intial pose2d, a list of pose2ds and an ending pose
     * Use "new Pose2d()"
     */


  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    //gets the state of the robot at the current time
    Trajectory.State reference = m_trajectory.sample(elapsed);
    //calculate the speeds
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * TODO
     * Have the robot drive aroun in teleop using either
     *    1. Constants speeds, can use the timer if you want
     *    2. Use a controller
     *    3. Use the keyboard (This requires so more setup so ask if you want help)
     * 
     */

     
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
}
