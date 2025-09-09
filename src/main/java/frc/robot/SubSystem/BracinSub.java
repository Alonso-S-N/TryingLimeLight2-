// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calcs;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BracinSub extends SubsystemBase {
  //private static final double gearRatio = 1;
  //private static final double degreesPerMotorRev = 360.0 / gearRatio;

  public final SparkMax armMotor = new SparkMax(Constants.m_Bracin, MotorType.kBrushless);
  public final SparkMax intakeMotor = new SparkMax(Constants.m_Intake, MotorType.kBrushless);
  //private final SparkClosedLoopController pidController = armMotor.getClosedLoopController();

  public BracinSub() {

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
  public void periodic() {
   // SmartDashboard.putNumber("Encoder Position", armMotor.getEncoder().getPosition());
   //SmartDashboard.putNumber("PID Output", armMotor.getAppliedOutput());
   SmartDashboard.putNumber("velocidadeBraceta", armMotor.getAppliedOutput());
  }
}