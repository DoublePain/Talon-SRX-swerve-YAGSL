// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// Swerve Subsystem Code yippee
public class SwerveSubsystem extends SubsystemBase {

  // Imports stuff from the JSON Files
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive;

  private TalonSRX frontLeftAngleMotor;
  private TalonSRX frontRightAngleMotor;
  private TalonSRX rearLeftAngleMotor;
  private TalonSRX rearRightAngleMotor;

  // Creates a New SwerveSubsystem
  public SwerveSubsystem() {
    
    frontLeftAngleMotor = new TalonSRX(DriveConstants.k_frontLeftAngleMotorID);
    frontRightAngleMotor = new TalonSRX(DriveConstants.k_frontRightAngleMotorID);
    rearLeftAngleMotor = new TalonSRX(DriveConstants.k_rearLeftAngleMotorID);
    rearRightAngleMotor = new TalonSRX(DriveConstants.k_rearRightAngleMotorID);
    

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // TURN OFF DURING COMPETITION BECAUSE IT * WILL *  SLOW YOUR ROBOT
    // (It's for displaying info in Shuffleboard)
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    
    // Initializes robot using the JSON Files with all the constants so you don't have to. Hooray!
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.k_maxSpeed);
    } 
      catch (Exception e) {
      throw new RuntimeException(e);
    }
    
    // Cosine Compensator makes your robot slower on some wheels. Set it to false if it drives funky
    swerveDrive.setCosineCompensator(false);

    configureAngleMotors(frontLeftAngleMotor);
    configureAngleMotors(frontRightAngleMotor);
    configureAngleMotors(rearLeftAngleMotor);
    configureAngleMotors(rearRightAngleMotor);
    
  }

  

  private void configureAngleMotors(TalonSRX motor) {
    motor.configFactoryDefault(); // Reset to factory settings
    motor.setInverted(false); // Set motor direction if needed (depends on wiring)

    // Configuring the mag encoder as the feedback device for angle control
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    motor.config_kF(0, 0.0); // Set feed-forward if necessary
    motor.config_kP(0, 1.0); // Set proportional control constant
    motor.config_kI(0, 0.0); // Set integral control constant
    motor.config_kD(0, 0.0); // Set derivative control constant

     // Right-side motors might need to be inverted
     if (motor == frontRightAngleMotor || motor == rearRightAngleMotor) {
      motor.setInverted(true);  // Try setting inverted for the right side
      
      //resetEncoders(); Try resetting encoders before enabling 

  }
  }

  // Command to drive the robot using translative values and heading as angular velocity.
  // translationX - Translation in the X direction. Cubed for smoother controls.
  // translationY - Translation in the Y direction. Cubed for smoother controls.
  // angularRotationX - Angular velocity of the robot to set. Cubed for smoother controls.
  // Returns Drive command.

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), 0.8),
        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
        true,
        false);
        

        double frontLeftAngle = calculateAngle(translationX.getAsDouble(), translationY.getAsDouble(), angularRotationX.getAsDouble(), "frontLeft");
        double frontRightAngle = calculateAngle(translationX.getAsDouble(), translationY.getAsDouble(), angularRotationX.getAsDouble(), "frontRight");
        double rearLeftAngle = calculateAngle(translationX.getAsDouble(), translationY.getAsDouble(), angularRotationX.getAsDouble(), "rearLeft");
        double rearRightAngle = calculateAngle(translationX.getAsDouble(), translationY.getAsDouble(), angularRotationX.getAsDouble(), "rearRight");

        System.out.println("Front Right Angle: " + frontRightAngle);
        System.out.println("Rear Right Angle: " + rearRightAngle);


        // Set the angles of the Talon SRX motors (position control)
        setMotorAngle(frontLeftAngleMotor, frontLeftAngle);  
        setMotorAngle(frontRightAngleMotor, frontRightAngle); 
        setMotorAngle(rearLeftAngleMotor, rearLeftAngle);
        setMotorAngle(rearRightAngleMotor, rearRightAngle);
       
        System.out.println("Front Left Angle: " + frontLeftAngle + " | Encoder Position: " + frontLeftAngleMotor.getSelectedSensorPosition());
        System.out.println("Front Right Angle: " + frontRightAngle + " | Encoder Position: " + frontRightAngleMotor.getSelectedSensorPosition());
        System.out.println("Rear Left Angle: " + rearLeftAngle + " | Encoder Position: " + rearLeftAngleMotor.getSelectedSensorPosition());
        System.out.println("Rear Right Angle: " + rearRightAngle + " | Encoder Position: " + rearRightAngleMotor.getSelectedSensorPosition());


    });

    
  }

  public void resetEncoders() {
    frontLeftAngleMotor.setSelectedSensorPosition(0);
    frontRightAngleMotor.setSelectedSensorPosition(0);
    rearLeftAngleMotor.setSelectedSensorPosition(0);
    rearRightAngleMotor.setSelectedSensorPosition(0);
    
    System.out.println("Encoders have been zeroed.");
}


private double calculateAngle(double translationX, double translationY, double angularVelocity, String wheelPosition) {
  // Robot's geometry
  //double L = SwerveConstants.k_trackwidth;
  //double W = SwerveConstants.k_wheelDiameter;

  // Compute the robot's velocity components
  double v_x = translationX;
  double v_y = translationY;
  double omega = angularVelocity;

  double x_offset = 0, y_offset = 0, offsetAngle = 0;

  switch (wheelPosition) {
      case "frontLeft":
          x_offset = SwerveConstants.k_FLXOffset;
          y_offset = SwerveConstants.k_FLYOffset;
          offsetAngle = SwerveConstants.k_WFLOffset; // Add front-left offset
          break;
      case "frontRight":
          x_offset = SwerveConstants.k_FRXOffset;
          y_offset = SwerveConstants.k_FRYOffset;
          offsetAngle = SwerveConstants.k_WFROffset; // Add front-right offset
          break;
      case "rearLeft":
          x_offset = SwerveConstants.k_RLXOffset;
          y_offset = SwerveConstants.k_RLYOffset;
          offsetAngle = SwerveConstants.k_WRLOffset; // Add rear-left offset
          break;
      case "rearRight":
          x_offset = SwerveConstants.k_RRXOffset;
          y_offset = SwerveConstants.k_RRYOffset;
          offsetAngle = SwerveConstants.k_WRROffset; // Add rear-right offset
          break;
  }

  double wheelVelocityX = v_x - omega * y_offset;
  double wheelVelocityY = v_y + omega * x_offset;

  double angle = Math.toDegrees(Math.atan2(wheelVelocityY, wheelVelocityX));

  // Add the offset to the calculated angle
  angle += offsetAngle;

  // Ensure the angle is within 0 to 360 degrees
  if (angle < 0) {
      angle += 360;
  } else if (angle >= 360) {
      angle -= 360;
  }

  return angle;
} 
  
private void setMotorAngle(TalonSRX motor, double angle) {
    // Convert the angle to encoder ticks if needed (based on motor setup)
    // Example: If your motor has 4096 ticks per revolution, you would convert the angle to ticks
    int angleTicks = (int) (angle * 4096 / 360); // Simple conversion for example
    motor.set(ControlMode.Position, angleTicks);

    //Trying to reverse our encoders!
    if (motor == frontRightAngleMotor || motor == rearRightAngleMotor) {
      // Reverse the direction of the encoder reading for the right-side motors
      motor.set(ControlMode.Position, -angleTicks); // Negative for reversal
  } else {
      motor.set(ControlMode.Position, angleTicks); // Normal angle for left-side motors
  }

  }
/* 
  private void SendMotorAngle(TalonSRX motor, double angle){

  }
  */
  @Override
  // This method will be called once per scheduler run
  public void periodic() {}

  @Override
  // This method will be called once per scheduler run during simulation
  public void simulationPeriodic() {}
}
