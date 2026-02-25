// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class BumpButton extends Command {

  private final Drivebase m_drivebase;
  private final Supplier<double[]> speedXY;
  private final DoubleSupplier rot;
  private final ProfiledPIDController thetaController = new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(60, 60));
  public Canandgyro m_gyro = new Canandgyro(46);
  
  public BumpButton(Supplier<double[]> speedXY, DoubleSupplier rot, Drivebase drivebase) {
    this.speedXY = speedXY;
    this.rot = rot;
    this.m_drivebase = drivebase;
    thetaController.setTolerance(Math.toRadians(1));
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);
  

   // Use addRequirements() here to declare subsystem dependencies
    addRequirements(this.m_drivebase);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }
   public double Routput;
    double angleThreshold = 0.03;
   
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xy = speedXY.get();
 
   
    

    //use isLocking boolean to turn 45 degrees if true and switch to normal drive if false

      double currentAngle = m_drivebase.getFieldAngle();
      double goalAngle = Math.toRadians(45);
      
      if (Math.toRadians(45) - currentAngle < Math.toRadians(45)) {

        goalAngle = Math.toRadians(45);

      } else if (Math.toRadians(135) - currentAngle < Math.toRadians(45)) {

          goalAngle = Math.toRadians(135);

      } else if (Math.toRadians(225) + currentAngle < Math.toRadians(45)) {

          goalAngle = Math.toRadians(225);

      }else if (Math.toRadians(315) + currentAngle < Math.toRadians(45)) {

          goalAngle = Math.toRadians(315);
      }

     
  

    //set defaultDrive to output
    SmartDashboard.putNumber("rotoutput", Routput);
    m_drivebase.defaultDrive(-xy[1], -xy[0], Routput);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
