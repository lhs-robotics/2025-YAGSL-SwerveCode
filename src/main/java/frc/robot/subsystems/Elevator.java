package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator {

    //Initializing motors and encoders 

    private final SparkFlex leadMotor;
    private final SparkFlex followMotor;

    


    // Encoder and PID Controller for the lead motor
    private final RelativeEncoder encoder;
    // private final SparkPIDController pidController;


    // Elevator Constants
    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.1;
    private static final double kFF = 0.0;
    private static final double kMinOutput = -1.0;
    private static final double kMaxOutput = 1.0;
    private static final double kEncoderToCm = 0.1; // change conversion to our robot

       // Predefined Heights (in cm)
       public enum ElevatorLevel {
        LEVEL1(10.0), // Level 1 height in cm
        LEVEL2(50.0), // Level 2 height in cm
        LEVEL3(100.0), // Level 3 height in cm
        LEVEL4(150.0); // Level 4 height in cm

        private final double heightCm;

        ElevatorLevel(double heightCm) {
            this.heightCm = heightCm;
        }

        public double getHeightCm() {
            return heightCm;
        }
    }

    public Elevator(int leadMotorID, int followMotorID) {
        // Initialize motors
        leadMotor = new SparkFlex(leadMotorID);
        followMotor = new SparkFlex(followMotorID);

        // Set the follower motor to follow the lead motor
        followMotor.follow(leadMotor, true); // true sets it as inverted

        // Configure encoder and PID controller
        encoder = leadMotor.getEncoder()
        pidController = leadMotor.getPIDController();

        // Configure PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        // Reset encoder position to 0
        encoder.setPosition(0.0);
    }

    /**
     * Command the elevator to a specific height in centimeters.
     *
     * @param heightCm The desired height in centimeters.
     */
    public void goToHeight(double heightCm) {
        double targetPosition = heightCm / kEncoderToCm; // Convert cm to encoder units
        pidController.setReference(targetPosition, CANSPARKFlex.ControlType.kPosition);
    }

    /**
     * Command the elevator to a predefined level.
     *
     * @param level The target elevator level.
     */
    public void goToLevel(ElevatorLevel level) {
        goToHeight(level.getHeightCm());
    }

    /**
     * Get the current elevator height in centimeters.
     *
     * @return The current height in centimeters.
     */
    public double getCurrentHeight() {
        return encoder.getPosition() * kEncoderToCm;
    }

    @Override
    public void periodic() {
        // Optionally, log the current height for debugging
        System.out.println("Elevator Height: " + getCurrentHeight() + " cm");
    }


}

