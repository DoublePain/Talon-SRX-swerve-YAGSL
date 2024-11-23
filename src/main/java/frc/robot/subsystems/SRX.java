package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

public class SRX { // Consider renaming this class to reflect TalonSRX usage
    private TalonSRX talonSRX;
    private Joystick joystick;
    private int deviceID; // Talon SRX device ID
    private int joystickPort; // Joystick port
    private double speedWhenPressed = 1; // Speed to set when the button is pressed

    // Constructor
    public SRX(int deviceID, int joystickPort) {
        this.deviceID = deviceID;
        this.joystickPort = joystickPort;
        this.talonSRX = new TalonSRX(this.deviceID); // Initialize TalonSRX
        this.joystick = new Joystick(this.joystickPort); // Initialize joystick
    }

    // Method to control the TalonSRX with a joystick button
    public void controlWithButton(int buttonNumber) {
        boolean buttonPressed = this.joystick.getRawButton(buttonNumber);
        if (buttonPressed) {
            this.talonSRX.set(ControlMode.PercentOutput, speedWhenPressed); // Set motor speed when button is pressed
        } else {
            this.talonSRX.set(ControlMode.PercentOutput, 0); // Stop motor when button is not pressed
        }
    }

    public void controlWithJoystick() {
        double speed = this.joystick.getY(); // Get Y-axis value
        this.talonSRX.set(ControlMode.PercentOutput, speed); // Set motor speed
    }
}