// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnToAngle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase { //This is the class where the subsystem is run
  private final WPI_TalonSRX centerMotor; //Instantiates a new motor
  private final WPI_TalonSRX motor; //Instantiates a new motor
  /* Creates a new Turret. */
  public Turret() { //constrcutor header
    motor = new WPI_TalonSRX(Constants.TurretConstants.talonPort); 
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

    centerMotor = new WPI_TalonSRX(Constants.TurretConstants.talonPort);
    centerMotor.configFactoryDefault();
    centerMotor.setInverted(false);
    centerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
  } // initiallizes the variables of the turret

  public void setAngle(double angle){ //Defines the angle that the turret is at currently
    motor.setSelectedSensorPosition(angle);
  }

  public double getAngle(){ //Returns the angle the turret is at currently in degrees
    return motor.getSelectedSensorPosition() * 360 / (4096.0);
    //4096 is the number of ticks in a circle,
    //so to convert from ticks to degrees
    //multiplying by 360 and dividing by 4096 converts it
  }

  public double getActualAngle(){ //Returns the angle the turret should be at in degrees
    return centerMotor.getSelectedSensorPosition() * 360 * 4 / (23 * 4096.0);
  }   

  public void spin(double speed){ //Makes the motor spin some speed from 0 to 1
    motor.set(ControlMode.PercentOutput, speed);
  }

  public double findSlippageAngle(){ //Returns the angle that the robot slipped in degrees
    return getActualAngle() - getAngle();
  }

  @Override
  public void periodic() { //Displays the angle the turrent is at on smartDashboard
    SmartDashboard.putNumber("Angle: ", getAngle());
    SmartDashboard.putNumber("Actual Angle: ", getActualAngle());
    SmartDashboard.putNumber("Angle Slipped: ", findSlippageAngle());
    motor.set(ControlMode.PercentOutput, -0.5*RobotContainer.getJoy().getY());
    //SmartDashboard.putNumber("Joystick Power: ", RobotContainer.getJoy().getY());
  }
}