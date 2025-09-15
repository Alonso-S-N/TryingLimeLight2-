package frc.robot.command.Auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.Vision;

public class FollowAprilTagTXTA extends Command {
    private final Drive drive;
    private final Vision vision;
    public final double targetArea;
    Timer timer = new Timer();
    private final double hysteresis = 0.5; // margem de tolerância (ajustar conforme teste)
    Joystick joyDeliciu;

    private boolean holdingPosition = false;

    public FollowAprilTagTXTA(Drive drive, Vision vision, double targetArea) {
        this.drive = drive;
        this.vision = vision;
        this.targetArea = targetArea;
        addRequirements(drive,vision);
    }

    @Override
    public void initialize() {
        drive.reqDrive();
        timer.reset();
        timer.start();
        holdingPosition = false;
    }

    @Override
    public void execute() {
        //se não tiver alvo apos 4 segundos, para
        if (!vision.hasTarget() && timer.get() > 4.0) {
            setDriveSpeeds(0, 0);
            holdingPosition = false; // reseta
            return;
        }

        // Se não tem alvo → gira por 2 segundos ou ate encontrar
        if (!vision.hasTarget() && timer.get() < 2.0) {
            setDriveSpeeds(-0.2, 0.2);
            holdingPosition = false; // reseta
            return;
        }
       // Se não tem alvo → gira por 2 segundos ou ate encontrar
        if (!vision.hasTarget() && timer.get() > 2.0 && timer.get() < 4.0) {
            setDriveSpeeds(0.2, -0.2);
            holdingPosition = false; // reseta
            return;
        }

        double tx = vision.getTx();
        double ta = vision.getTa();


        if (holdingPosition && ta >= (targetArea - hysteresis)) {   
            setDriveSpeeds(0, 0);
            return;
        }

       
        if (ta >= targetArea) {
            holdingPosition = true;
            setDriveSpeeds(0, 0);
            return;
        }

        holdingPosition = false;

        double kP_turn = 0.03;
        double kP_forward = 0.1;

        double turn = tx * kP_turn; 
        double forward = (targetArea - ta) * kP_forward;

        // Limita a velocidade máxima para 50%
        forward = Math.max(-0.5, Math.min(0.5, forward));

        double left = forward + turn;
        double right = forward - turn;

        // Saturação final para [-0.5, 0.5]
        left = Math.max(-0.5, Math.min(0.5, left));
        right = Math.max(-0.5, Math.min(0.5, right));

        setDriveSpeeds(left, right);
    }

    @Override
    public boolean isFinished() {
        return false; // só termina pelo AutonomousCommand (timer)
    }

    @Override
    public void end(boolean interrupted) {
        setDriveSpeeds(0, 0);
    }

    private void setDriveSpeeds(double left, double right) {
        drive.m_leftDrive.set(ControlMode.PercentOutput, left);
        drive.m_leftDrive2.set(ControlMode.PercentOutput, left);
        drive.m_rightDrive.set(ControlMode.PercentOutput, right);
        drive.m_rightDrive2.set(ControlMode.PercentOutput,right);
    }
}
