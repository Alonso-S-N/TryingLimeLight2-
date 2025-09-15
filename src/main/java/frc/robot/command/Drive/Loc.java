package frc.robot.command.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.Vision;
import frc.robot.Calcs;
import frc.robot.Calcs.DriveSpeeds;

public class Loc extends Command {
  
  private final Drive driveSubsystem;
  private final Joystick joyDeliciu;
  DriveSpeeds speeds;
  private boolean followTagMode = false;
  double tx,ta;

   private final Vision vision;
   public final double targetArea;
  
    private double B_Speed = 0;
    private boolean a, b, x;

    public Loc(Drive driveSubsystem,Joystick joyDeliciu, Vision vision, double targetArea) {
      this.driveSubsystem = driveSubsystem;
      this.joyDeliciu = joyDeliciu;
      this.vision = vision;
      this.targetArea = targetArea;
    addRequirements(driveSubsystem,vision);
  }

  @Override
  public void initialize() {
    driveSubsystem.reqDrive();
  }

  @Override
  public void execute() {
    Smart();
    button();
    MainControl();
    SmartDashboard.putString("Loc Status", "Rodando");
  }

  private void setDriveSpeeds(double left, double right) {
    driveSubsystem.m_leftDrive.set(ControlMode.PercentOutput, left);
    driveSubsystem.m_leftDrive2.set(ControlMode.PercentOutput, left);
    driveSubsystem.m_rightDrive.set(ControlMode.PercentOutput, right);
    driveSubsystem.m_rightDrive2.set(ControlMode.PercentOutput, right);
  }

  private void stopDrive() {
    setDriveSpeeds(0, 0);
  }

  public void button(){
    a = joyDeliciu.getRawButton(Constants.a);
    b = joyDeliciu.getRawButton(Constants.b);
    x = joyDeliciu.getRawButton(Constants.x);

    if (joyDeliciu.getRawButtonPressed(10)) {
      followTagMode = false;
  }
    if (joyDeliciu.getRawButtonPressed(9)) {
      followTagMode = true;
  }

    if (a) B_Speed = 0.25;
    else if (b) B_Speed = 0.5;
    else if (x) B_Speed = 1.0;
  }

  public void MainControl() {
    double X = joyDeliciu.getX();
    double Y = joyDeliciu.getY();
    double X1 = joyDeliciu.getRawAxis(Constants.X1);
    double Y2 = joyDeliciu.getRawAxis(Constants.Y2);

    if (joyDeliciu.getPOV() != Constants.povDeadZone) {
        speeds = Calcs.calculatePovDrive(joyDeliciu, B_Speed);
        setDriveSpeeds(speeds.left, speeds.right);
    }
    else if (Math.abs(X) >= Constants.deadZone || Math.abs(Y) >= Constants.deadZone
          || Math.abs(X) < Constants.NegativeDeadZone || Math.abs(Y) < Constants.NegativeDeadZone) {
        speeds = Calcs.calculateAnalogDrive(joyDeliciu, B_Speed); 
        setDriveSpeeds(speeds.left, speeds.right);
    }
    else if (Math.abs(X1) >= Constants.deadZone || Math.abs(Y2) >= Constants.deadZone
          || Math.abs(X1) < Constants.NegativeDeadZone || Math.abs(Y2) < Constants.NegativeDeadZone) {
        speeds = Calcs.calculateAnalogDrive2(joyDeliciu, B_Speed);
        setDriveSpeeds(speeds.left, speeds.right);
    }
    else if (followTagMode && vision.hasTarget()) {
        IndoPraTag();  // já define motores e speeds
    }
    else {
        stopDrive();
        speeds = null;
    }
}

  public void Smart(){
    if (speeds != null) {
      SmartDashboard.putNumber("Left Speed", speeds.left);
      SmartDashboard.putNumber("Right Speed", speeds.right);
    } else {
      SmartDashboard.putNumber("Left Speed", 0);
      SmartDashboard.putNumber("Right Speed", 0);
    }
    SmartDashboard.putBoolean("Button A", a);
    SmartDashboard.putBoolean("Button B", b);
    SmartDashboard.putBoolean("Button X", x);
    SmartDashboard.putNumber("VelB", B_Speed);
    SmartDashboard.putBoolean("AprilTagMode", followTagMode);
    SmartDashboard.putBoolean("Has Target", vision.hasTarget());
  }

  public void IndoPraTag() {
    double kP_turn = 0.03;
    double kP_forward = 0.1;
    double hysteresis = 0.5; // margem de tolerância

    double tx = vision.getTx();
    double ta = vision.getTa();

    double turn = tx * kP_turn; 
    double forward = (targetArea - ta) * kP_forward;

    forward = Math.max(-0.5, Math.min(0.5, forward));

    double left = forward + turn;
    double right = forward - turn;

    left = Math.max(-0.5, Math.min(0.5, left));
    right = Math.max(-0.5, Math.min(0.5, right));

    // Só para se já chegou bem perto
    if (ta >= (targetArea - hysteresis)) {
        left = 0;
        right = 0;
    }

    setDriveSpeeds(left, right);
    speeds = new Calcs.DriveSpeeds(left, right);
}

  @Override
  public void end(boolean interrupted) {
    stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
