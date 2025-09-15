package frc.robot.command.Auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SubSystem.BracinSub;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.Vision;


public class AutonomousCommand extends FollowAprilTagTXTA {
  private double vel;

  private boolean finished,recuando = false;

  BracinSub braceta;

  Timer SensorTime = new Timer();

  Vision vision;

  double targetArea;


  
  Timer timer = new Timer();

  Timer recuoTimer = new Timer();
  private Drive driveSubsystem;
  public AutonomousCommand(Drive driveSubsystem, BracinSub braceta, Vision vision, double targetArea) {
    super(driveSubsystem, vision, targetArea);
    this.driveSubsystem = driveSubsystem;
    this.braceta = braceta;
    this.vision = vision;
    this.targetArea = Constants.targetArea;


    addRequirements(driveSubsystem,braceta,vision);
  }

  public void setSpeedAuto(){
    driveSubsystem.m_leftDrive.set(ControlMode.PercentOutput, vel);
    driveSubsystem.m_leftDrive2.set(ControlMode.PercentOutput, vel);
    driveSubsystem.m_rightDrive.set(ControlMode.PercentOutput, vel);
    driveSubsystem.m_rightDrive2.set(ControlMode.PercentOutput, vel);
  }

  public void mexe() {
    vel = Constants.autonomousLoc;
    setSpeedAuto();
} 

private void stopDrive() {
    vel = 0;
    setSpeedAuto();

    System.out.println("STOP DRIVE CALLED!");
}
  
  @Override
  public void initialize() {
    timer.start();
    driveSubsystem.reqDrive();

  } 

  @Override
  public void execute() {
  super.execute();
}

  @Override
  public void end(boolean interrupted) {
    stopDrive();
  }

  @Override
  public boolean isFinished() {
    if (timer.get() >= Constants.autonomousTime){
      finished = true;
    }
  SmartDashboard.putBoolean("Auto Finished", finished);
  return finished;
  }
}
