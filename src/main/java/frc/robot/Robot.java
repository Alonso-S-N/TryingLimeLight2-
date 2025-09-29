// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SubSystem.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.SubSystem.Drive;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // private Loc locCommand;
  
  Timer timer = new Timer();
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private Command BracetaCommand;

  
      /**
       * This function is run when the robot is first started up and should be used for any
       * initialization code.
       */
      public Robot() {
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
      // autonomous chooser on the dashboard
    }
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
     m_autonomousCommand = m_robotContainer.getAutonomousCommand();
     BracetaCommand = m_robotContainer.getBracinCommand();
     Logger.recordMetadata("ProjectName", "Binga"); 
     Logger.recordMetadata("RuntimeType", RobotBase.getRuntimeType().toString());
   
     if (RobotBase.isSimulation()) {
       // Salva no ./logs (dentro da pasta do projeto) + envia via NT4
       Logger.addDataReceiver(new WPILOGWriter("logs"));
       Logger.addDataReceiver(new NT4Publisher());
     } else {
       // Robo real → salva no USB (/U) + envia via NT4
       Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
       Logger.addDataReceiver(new NT4Publisher());
     }
     //Logger.start();
}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel(); // cancela se ainda estiver rodando
    }
  }

  @Override
  public void teleopPeriodic() {
  }
}

