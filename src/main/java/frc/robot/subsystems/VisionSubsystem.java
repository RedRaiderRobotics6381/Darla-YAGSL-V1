package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;

public class VisionSubsystem {
    
    public static PhotonCamera camObj = new PhotonCamera("camObj"); // Create a new PhotonCamera object
    public static PhotonCamera camAprTgLow = new PhotonCamera("camAprTgLow"); // Create a new PhotonCamera object

    private LEDsSubSystem m_LEDsSubSystem;

    public VisionSubsystem(LEDsSubSystem ledsSubsystem) {
        m_LEDsSubSystem = ledsSubsystem;
        camObj.setDriverMode(false); // Set the camera to driver mode
        camAprTgLow.setDriverMode(false); // Set the camera to driver mode
    }

    public void periodic() {
        camObj.getLatestResult();
        camAprTgLow.getLatestResult();
        SmartDashboard.putBoolean("camObj Connected",camObj.isConnected()); // Check if the camera is connected
        SmartDashboard.putBoolean("camAprTgLow Connected",camAprTgLow.isConnected()); // Check if the camera is connected
        SmartDashboard.putBoolean("camObj has Targets",camObj.getLatestResult().hasTargets()); // Check if the camera has targets
        SmartDashboard.putBoolean("camAprTgLow has Targets",camAprTgLow.getLatestResult().hasTargets()); // Check if the camera has targets
    }

    /**
     * Checks if there are any targets detected by PhotonVision.
     * 
     * @return true if targets are found, false otherwise.
     */
    public boolean watchForNote(){
        boolean hasTargets = false;
        var result = camObj.getLatestResult(); //Get the latest result from PhotonVision
        hasTargets = result.hasTargets(); // Check if the latest result has any targets.
        if (hasTargets == true){
            m_LEDsSubSystem.fadeEffect(150, 255);
            //LEDsSubSystem.setSolidLED(150, 255, 50);
        } else {
            if (DriverStation.getAlliance().get() == Alliance.Blue) { 
                // LEDsSubSystem.fadeEffect(120, 255);
                m_LEDsSubSystem.scanEffect(120, 255, 255);
                // m_LEDsSubSystem.strobeEffect(120, 255, 255);
                //LEDsSubSystem.setSolidLED(120, 255, 50);
            } else {
                // LEDsSubSystem.fadeEffect(0, 255);
                m_LEDsSubSystem.scanEffect(0, 255, 255);
                // m_LEDsSubSystem.strobeEffect(0, 255, 255);
                //LEDsSubSystem.setSolidLED(0, 255, 50);
            }
        }
        return hasTargets;
    }
}
