package frc.robot.SubSystem;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.system.plant.DCMotor;

import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final Field2d field = new Field2d();
  private final NetworkTableEntry poseEntry;
  public EncoderSim leftEncoderSim;
  public EncoderSim rightEncoderSim;
  public ADXRS450_GyroSim gyroSim;
  private DifferentialDrivetrainSim driveSim;
  private double _debugX = 0.0;
  private double _lastTime = 0.0;
  private final WPI_VictorSPX m_leftLeader  = new WPI_VictorSPX(Constants.LMot);
  private final WPI_VictorSPX m_rightLeader = new WPI_VictorSPX(Constants.RMot);
  private final WPI_VictorSPX m_leftFollower  = new WPI_VictorSPX(Constants.LMot2);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(Constants.RMot2);

  public final Encoder leftEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
  private final Encoder rightEncoder = new Encoder(6, 7, true, Encoder.EncodingType.k4X);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final double diametroRoda = 0.06; // 6 cm
  private final DifferentialDriveOdometry odometry;

  public Drive() {
    reqDrive();
    resetEncoders();
    gyro.reset();

    SmartDashboard.putData("Field", field);
     poseEntry = NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getEntry("RobotPose");

     if (RobotBase.isSimulation()) {
    leftEncoderSim  = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    gyroSim         = new ADXRS450_GyroSim(gyro);
    _lastTime = Timer.getFPGATimestamp();
    }

  odometry = new DifferentialDriveOdometry(getHeading(),leftEncoder.getDistance(), rightEncoder.getDistance());
     driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(4),   // Tipo e número de motores (ajuste pro VictorSPX/775pro se precisar)
      15.4,                // Relação de engrenagem
      7.5,                 // Inércia do chassi
      40.0,                // Massa do robo (kg)
      0.03,                // Raio da roda (m)
      0.75,                 // Largura entre rodas (trackwidth em m)
      null                 // Random noise 
  );
  }

  @Override
  public void simulationPeriodic() {
    if (driveSim != null) {

      driveSim.setInputs(
          m_leftLeader.get() * 12.0,
          m_rightLeader.get() * 12.0
      );

      driveSim.update(0.02);

      leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
      rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
      leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
      rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
      gyroSim.setAngle(-driveSim.getHeading().getDegrees());
    }
  }

  @Override
  public void periodic() {
    odometry.update( getHeading(),leftEncoder.getDistance(),rightEncoder.getDistance());
    Pose2d pose = odometry.getPoseMeters();
    
    field.setRobotPose(pose);
    poseEntry.setDoubleArray(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});

    Logger.recordOutput("Drive/Pose", pose);
    Logger.recordOutput("Drive/LeftDistance", leftEncoder.getDistance());
    Logger.recordOutput("Drive/RightDistance", rightEncoder.getDistance());
    Logger.recordOutput("Drive/LeftOutput", m_leftLeader.get());
    Logger.recordOutput("Drive/RightOutput", m_rightLeader.get());
  }

  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    gyro.reset(); 
    odometry.resetPosition(getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void reqDrive() {
    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_rightLeader.setInverted(true);
    m_leftLeader.setInverted(false);

    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);

    double distancePerPulse = (Math.PI * diametroRoda) / 2048;
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);
  }

  public void rawTank(double left, double right) {
    m_leftLeader.set(left);
    m_rightLeader.set(right);

    Logger.recordOutput("Drive/LeftSetpoint", left);
    Logger.recordOutput("Drive/RightSetpoint", right);
  }

  public void stop() {
    m_leftLeader.stopMotor();
    m_rightLeader.stopMotor();
  }
}