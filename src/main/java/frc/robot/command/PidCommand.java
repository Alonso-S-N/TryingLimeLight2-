// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystem.BracinSub;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PidCommand extends Command {
  private final BracinSub braceta;
  private final double ANgulinQnoisQuer = Constants.angulinMax;
  private final double ANgulinQnoisQuer2 = Constants.angulinMin;

  private Joystick joyDeliciu = new Joystick(0);
  
    public PidCommand(BracinSub braceta, Joystick joyDeliciu) {
     this.braceta = braceta;
     this.joyDeliciu = joyDeliciu;
   addRequirements(braceta);
  }


  @Override
  public void initialize() {
  }

  public void mexerBracinSlk(){
  if (joyDeliciu.getRawAxis(Constants.RT) > 0.1){
    braceta.MexePruLado();
  } else if (joyDeliciu.getRawAxis(Constants.LT) > 0.1){
    braceta.MexePruOutro();
  } else {
    braceta.StopBraceta();
  }
   //if (joyDeliciu.getRawButton(Constants.RB)){
     // braceta.MexePru(ANgulinQnoisQuer);
   // } else if (joyDeliciu.getRawButton(Constants.LB)){
     // braceta.MexePru(ANgulinQnoisQuer2);
    //}  
  }

  public void mexerIntakeSlk(){
    if (joyDeliciu.getRawButton(Constants.RB)){
      braceta.Pegar();
    } else if (joyDeliciu.getRawButton(Constants.LB)){
      braceta.Cuspir();
    } else {
      braceta.stopIntake();
    }
  }
  
  @Override
  public void execute() {
    joyDeliciu.getRawButton(Constants.RB);
    joyDeliciu.getRawButton(Constants.LB);
    joyDeliciu.getRawAxis(Constants.RT);
    joyDeliciu.getRawAxis(Constants.LT);
    
   mexerBracinSlk();
   mexerIntakeSlk();

   SmartDashboard.putNumber("velocidade Braceta", braceta.armMotor.getAppliedOutput());
   SmartDashboard.putNumber("velocidade Intake", braceta.intakeMotor.getAppliedOutput());
  }


  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
