// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calcs;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class BracinSub extends SubsystemBase {
  //private static final double gearRatio = 1;
  //private static final double degreesPerMotorRev = 360.0 / gearRatio;

  public final SparkMax armMotor = new SparkMax(Constants.m_Bracin, MotorType.kBrushless);
  public final SparkMax intakeMotor = new SparkMax(Constants.m_Intake, MotorType.kBrushless);
  Encoder armEncoder = new Encoder (0, 1, false, Encoder.EncodingType.k4X);
  Encoder IntakeEncoder = new Encoder (2, 3, false, Encoder.EncodingType.k4X);
  EncoderSim armEncoderSim = new EncoderSim(armEncoder);
  EncoderSim intakeEncoderSim = new EncoderSim(IntakeEncoder);
     private final LoggedMechanism2d mech = new LoggedMechanism2d(0.7, 0.5);
    private final LoggedMechanismRoot2d base = mech.getRoot("Base", 0.6, 0.7);
    private final LoggedMechanismLigament2d armLig = base.append(
        new LoggedMechanismLigament2d("Arm", 0.5, 90,10,new Color8Bit(192,192,192)));
    private final LoggedMechanismLigament2d intakeLig = armLig.append(
        new LoggedMechanismLigament2d("Intake", 0.3, 90,5,new Color8Bit(255,0,0)));


  //private final SparkClosedLoopController pidController = armMotor.getClosedLoopController();

  public BracinSub() {
    armEncoder.reset();
    IntakeEncoder.reset();
    armEncoder.setDistancePerPulse(1.0/2048.0);
    IntakeEncoder.setDistancePerPulse(1.0/2048.0);


   

    //SparkMaxConfig cfg = new SparkMaxConfig();

    //cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);

    //armMotor.getEncoder().setPosition(Constants.sensorPos);
    //cfg.encoder.positionConversionFactor(1.0);
    //cfg.closedLoop
        //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //.p(0.001).i(0.0).d(0.001)
        //.outputRange(-0.5, 0.5);

    //armMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void MexePruLado() {
    armMotor.set(0.10);
    //pidController.setReference(ANgulinQnoisQuer, ControlType.kPosition);
  }

  public void MexePruOutro(){
    armMotor.set(-0.10);
  }

  public void StopBraceta(){
    armMotor.set(0.0);
  }
  
  public void Cuspir(){
    intakeMotor.set(0.7);
  }
  public void Pegar(){
    intakeMotor.set(-0.7);
  }
  public void stopIntake(){
    intakeMotor.set(0.0);
  }

  @Override
  public void simulationPeriodic() {
      double motorOut = armMotor.get();
      double simulatedRate = motorOut * 5.0; 
      armEncoderSim.setRate(simulatedRate);
      armEncoderSim.setDistance(armEncoder.getDistance() + simulatedRate * 0.02);
  }

  @Override
  public void periodic() {
    IntakeEncoder.getDistance();
    armEncoder.getDistance(); 
    double armAngleDeg = armEncoder.getDistance() * 360.0; // em graus
    armLig.setAngle(armAngleDeg);
    double intakeAngleDeg = IntakeEncoder.getDistance() * 360.0;
    intakeLig.setAngle(intakeAngleDeg);

    double armAngleRad = armEncoder.getDistance() * 2 * Math.PI; 
    Logger.recordOutput("Arm/Mechanism", new edu.wpi.first.math.geometry.Rotation2d(armAngleRad));
    Logger.recordOutput("Arm/PositionTicks", armEncoder.getDistance());
    Logger.recordOutput("Arm/Mechanism2d", mech);


    Logger.recordOutput("Arm/VelocityTicksPerSec", armEncoder.getRate());


    Logger.recordOutput("Arm/AppliedOutput", armMotor.getAppliedOutput());

    // Intake
    Logger.recordOutput("Intake/AppliedOutput", intakeMotor.getAppliedOutput());
    Logger.recordOutput("Intake/PositionTicks", IntakeEncoder.getDistance());
  }
}
