package frc.robot.SubSystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class Drive extends SubsystemBase {

  private final Encoder encodin = new Encoder(0,1, false, Encoder.EncodingType.k4X);
  // Motores
  public final VictorSPX m_leftDrive  = new VictorSPX(Constants.LMot);
  public final VictorSPX m_rightDrive = new VictorSPX(Constants.RMot);
  public final VictorSPX m_rightDrive2 = new VictorSPX(Constants.RMot2);
  public final VictorSPX m_leftDrive2  = new VictorSPX(Constants.LMot2);

  private final double diametroRoda = 0.06; // 6 cm

  public Drive() {
    // Configura motores e Encoder
    reqDrive();
  }

  @Override
  public void periodic() {
   Smart();
  }

  public double getDistance() {
    return encodin.getDistance();
  }

  public void resetEncoders() {
    encodin.reset();
  }

  public void reqDrive() {
    m_rightDrive.setInverted(true);
    m_rightDrive2.setInverted(true);
    m_leftDrive.setInverted(false);
    m_leftDrive2.setInverted(false);

    m_leftDrive.setNeutralMode(NeutralMode.Brake);
    m_leftDrive2.setNeutralMode(NeutralMode.Brake);
    m_rightDrive.setNeutralMode(NeutralMode.Brake);
    m_rightDrive2.setNeutralMode(NeutralMode.Brake);

     // Configura encoder
     double distancePerPulse = (Math.PI * diametroRoda) / 2048;
     encodin.setDistancePerPulse(distancePerPulse);
     encodin.setReverseDirection(true);
  }

  public void Smart(){
    SmartDashboard.putNumber("Encoder Distance", getDistance());
    SmartDashboard.putNumber("Encoder Pulses", encodin.getRaw());
  }
}
