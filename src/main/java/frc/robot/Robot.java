

package frc.robot;

/* =============================================== IMPORTING =========================================================== */

// import all of rev robotics and wpi lib classes that are needed
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// start of Robot class
public class Robot extends TimedRobot
{

/* ============================================== AUTO SELECTIONS ====================================================== */

// autonomous selection options:
private static final String nothingAuto = "do nothing";
private static final String launchAndDrive = "launch drive";                      // 15 in. w/o bumper (kinda central) -> almost 9.5 in.
private static final String launch = "launch";
private static final String drive = "drive";
private static final String launchAndTurnRed = "launch and turn red";
private static final String launchAndTurnBlue = "launch and turn blue";

private String autoSelected;
private final SendableChooser<String> chooser = new SendableChooser<>();          // SendableChooser class that was imported from wpilib

/* ===================================== MOTOR CONTROLLER and CONTROLLER INSTANCES ==================================== */

// drive motor controller instances
// CANSparkBase objects: CANSparkMax(int deviceId, MotorType type) -> MotorType type from MotorType class that is imported from rev robotics (kBrushed for brushed motors like CIMs and RedLines, kBrushless for brushless motors like NEOs)
CANSparkBase leftFront = new CANSparkMax(1, MotorType.kBrushed);         // CIM, SparkMax 1
CANSparkBase leftRear = new CANSparkMax(2, MotorType.kBrushed);          // CIM, SparkMax 2
CANSparkBase rightFront = new CANSparkMax(3, MotorType.kBrushed);        // CIM, SparkMax 3
CANSparkBase rightRear = new CANSparkMax(4, MotorType.kBrushed);         // CIM, SparkMax 4

// class to control drivetrain
DifferentialDrive drivetrain;

// launcher motor controller instances (for motor controllers other than SparkMax- like VictorSPX- import different motor controller classes)
// CANSparkBase launchWheel = new CANSparkMax(6, MotorType.kBrushed);              // CIM, SparkMax 6
CANSparkBase feedWheel = new CANSparkMax(5, MotorType.kBrushed);          // CIM, SparkMax 5

// launcher motor controller instances in case we change to NEOs
CANSparkBase launchWheel = new CANSparkMax(6, MotorType.kBrushless);      // NEO, SparkMax 6
// CANSparkBase feedWheel = new CANSparkMax(5, MotorType.kBrushless);              // NEO, SparkMax 5

// roller claw motor controller instance (currently RedLine)
CANSparkBase rollerClaw = new CANSparkMax(8, MotorType.kBrushed);         // RedLine, SparkMax 8

// climber motor controller instance (currently with NEO)
CANSparkBase climber = new CANSparkMax(7, MotorType.kBrushless);          // NEO, SparkMax 7

// climber motor controller instance in case changed to CIM
// CANSparkBase climber = new CANSparkMax(7, MotorType.kBrushed);                   // CIM, SparkMax 7

// Joystick (DriveTrain) instance
// Joystick Objects: Joystick(int port)
Joystick driverController = new Joystick(0);

// Xbox Controller (Shooter/Claw/Climber) instance
// XboxController class extends Joystick, so use same object argument
XboxController manipController = new XboxController(1);

/* ================================================ NUMBER SETTINGS ================================================= */

// amps individual drivetrain motor can use
static final int DRIVE_CURRENT_LIMIT_A = 60;

// amps feeder motor can use
static final int FEEDER_CURRENT_LIMIT_A = 80;

// percent output to run feeder when expelling note
static final double FEEDER_OUT_SPEED = 1.0;                                       // 100%

// percent output to run the feeder when intaking note
static final double FEEDER_IN_SPEED = -0.4;                                       // 40% in opposite direction

// percent output for amp or drop note (BASED ON POLYCARB BENT)
static final double FEEDER_AMP_SPEED = 0.4;                                       // 40% 

// amps launcher motor can use (CIM)
// static final int LAUNCHER_CURRENT_LIMIT_A = 80;

// amps launcher motor can use in case change to NEOs
static final int LAUNCHER_CURRENT_LIMIT_A = 60;

// percent output to run launcher motor when intaking and expelling note
static final double LAUNCHER_SPEED = 1.0;                                         // 100%

// in case want to make the above speeds differ for intaking and expelling note: (not used in the program)
static final double LAUNCHER_IN_SPEED = -1.0;                                     // 100%
static final double LAUNCHER_OUT_SPEED = 1.0;                                     // 100%

// percent output for scoring in amp or dropping note (BASED ON POLYCARB BENT)
static final double LAUNCHER_AMP_SPEED = 0.17;                                    // 17%

// percent output for roller claw
static final double CLAW_OUTPUT_POWER = 0.5;                                      // 50%

// percent output to help retain notes in claw
static final double CLAW_STALL_POWER = 0.1;                                       // 10%

// percent output to power the climber
static final double CLIMBER_OUTPUT_POWER = 1.0;                                   // 100%

// how long in sec the launcher motor should spin when button pressed (for set time button)
static final double LAUNCHER_TIME = 5.0;                                         // launcher runs for 5 seconds

// how long in sec the feeder motor should spin when button pressed (for set time button)
static final double FEEDER_TIME = 2.0;                                           // feeder runs for 2 seconds

// time setting for both shooter code
static final double BOTH_SHOOTER_LAUNCHER_DELAY = 0.3;                           // 2 second after button pressed - feeder turns on and launcher continues
static final double BOTH_SHOOTER_END_TIME = 0.5;                                 // 4 seconds after button pressed - launcher and feeder turn off (end of shooting)

// time setting for climber (all or partial)
static final double FULL_CLIMBER_SECONDS = 0.5;                                  // 5 seconds of running climber
static final double PARTIAL_CLIMBER_SECONDS = 0.25;                              // 2.5 seconds of running

static boolean TOP_MOTOR_RUNNING = false;

/* ============================================= BUTTON SETTINGS ===================================================== */

// shooter buttons
// static final int SHOOTER_AMP_HOLD_OUT = 0;                                      // on Xbox Controller: 
static final int BOTH_MOTORS_SHOOTER_IN = 7;                                    // on Xbox Controller: LT; intake motors
// static final int FEEDER_TIME_OUT = 0;                                           // on Xbox Controller: 
// static final int FEEDER_HOLD_IN = 0;                                            // on Xbox Controller: 
static final int FEEDER_HOLD_OUT = 8;                                           // on Xbox Controller: RT; just spin feeder wheel
// static final int LAUNCHER_TIME_OUT = 0;                                         // on Xbox Controller: 
static final int LAUNCHER_HOLD_OUT = 3;                                         // on Xbox Controller: B; just spin launcher wheel
static final int FULL_SHOOTING_MECHANISM = 1;                                   // on Xbox Controller: X; spin feeder for 2 sec, then launcher
static final int ON_LONG_BUTTON = 4;                                             // on Xbox Controller: Y; spin on until press A
static final int OFF_LONG_BUTTON = 2;                                            // on Xbox Controller: A; turn off launch wheel

// roller claw buttons
static final int ROLLER_CLAW_IN = 5;                                            // on Xbox Controller: LB; claw in
static final int ROLLER_CLAW_OUT = 6;                                           // on Xbox Controller: RB; claw out
// static final int ROLLER_CLAW_PASSIVE_OUT = 0;                                   // on Xbox Controller: 

// climber buttons- NOT USING BECAUSE USING POV FOR CLIMBER
// static final int CLIMBER_UP = 4;                                                // on Xbox Controller: Y; climber up in case limit breaks
// static final int CLIMBER_DOWN = 2;                                              // on Xbox Controller: A; climber down in case limit breaks
// static final int CLIMBER_ALL_UP = 0;                                            // on Xbox Controller: 
// static final int CLIMBER_PARTIAL_UP = 0;                                        // on Xbox Controller: 

/* ============================================= ROBOT INIT METHOD =================================================== */

@Override               // overrides the robotInit() method that already exists with the one below
// start of robotInit() method, occurs when robot first started up, use for initialization code
public void robotInit()
{
  // method from SendableChooser class: addOption(String name, String object)
  chooser.setDefaultOption("do nothing", nothingAuto);
  chooser.addOption("launch note and drive", launchAndDrive);
  chooser.addOption("launch", launch);
  chooser.addOption("drive", drive);
  chooser.addOption("launch note and turn red", launchAndTurnRed);
  chooser.addOption("launch note and turn blue", launchAndTurnBlue);

  // method from SmartDashboard class: putData(String key, Sendable data)
  SmartDashboard.putData("Auto choices", chooser);

  // apply limits to drivetrain motors (setSmartCurrentLimit(int limit) from CANSparkBase class)
  // update limit on line 74 above
  leftRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
  leftFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
  rightRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
  rightFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);

  // tells rear wheels to follow same commands as front wheels
  // follow(CANSparkBase leader) method from CANSparkBase class
  leftRear.follow(leftFront);
  rightRear.follow(rightFront);

  // one side of drivetrain must be inverted (motors facing opposite directions)
  // CANSparkBase method setInverted(boolean isInverted)
  leftFront.setInverted(true);                  // also applies to leftRear because following
  rightFront.setInverted(false);                // also applies to rightRear because following

  // DifferentialDrive object arguments: (MotorController leftMotor, MotorController rightMotor)
  drivetrain = new DifferentialDrive(leftFront, rightFront);

  // change direction launcher wheels are spinning
  feedWheel.setInverted(true);
  launchWheel.setInverted(false);

  // apply limit to launching mechanism
  feedWheel.setSmartCurrentLimit(FEEDER_CURRENT_LIMIT_A);
  launchWheel.setSmartCurrentLimit(LAUNCHER_CURRENT_LIMIT_A);

  // change direction roller claw is spinning
  rollerClaw.setInverted(true);

  // apply limit to roller claw
  rollerClaw.setSmartCurrentLimit(60);

  // change direction climber is spinning
  climber.setInverted(false);

  // apply limit to climber 
  climber.setSmartCurrentLimit(60);

  // set roller claw to idle in brake or coast mode (brake best)
  // to change to coast mode, change kBrake to kCoast
  rollerClaw.setIdleMode(IdleMode.kBrake);

  // set climber to idle in brake or coast mode (brake best)
  // to change to coast mode, change kBrake to kCoast
  climber.setIdleMode(IdleMode.kBrake);

} // end of robotInit() method

/* ============================================== ROBOT PERIODIC METHOD ================================================ */

@Override
// start of robotPeriodic() method, called every 20 ms (no matter mode), use for diagnostics that you want to run during disabled, auto, teleop, and test mode
public void robotPeriodic()
{

  // SmartDashboard class putNumber(String key, double value)
  // Timer class getFPGATimestamp() will return the time from the FPGA hardware clock in seconds since FPGA started
  SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());

} // end of robotPeriodic() method

/* =================================================== AUTO CONSTANTS ================================================== */

// delayed action starts X seconds into auto period
// time action will perform an action for X amount of seconds

double AUTO_LAUNCH_DELAY_S;
double AUTO_DRIVE_DELAY_S;
double AUTO_TURN_DELAY_S;
double AUTO_TURN_2_DELAY_S;

double AUTO_DRIVE_TIME_S;

double AUTO_DRIVE_SPEED;
double AUTO_LAUNCHER_SPEED;

double AUTO_Z_TURN;
double AUTO_Z_TURN_2;

double autonomousStartTime;

/* ==================================================== AUTO INIT METHOD =============================================== */

@Override
// start of autonomousInit() method
public void autonomousInit()
{

  autoSelected = chooser.getSelected();

  leftRear.setIdleMode(IdleMode.kBrake);
  leftFront.setIdleMode(IdleMode.kBrake);
  rightRear.setIdleMode(IdleMode.kBrake);
  rightFront.setIdleMode(IdleMode.kBrake);

  AUTO_LAUNCH_DELAY_S = 2.0;
  AUTO_DRIVE_DELAY_S = 3.0;
  AUTO_TURN_DELAY_S = 1.0;
  AUTO_TURN_2_DELAY_S = 0.5;

  AUTO_DRIVE_TIME_S = 4.0;
  AUTO_DRIVE_SPEED = -0.5;
  AUTO_LAUNCHER_SPEED = 1.0;

  AUTO_Z_TURN = 0.0;
  AUTO_Z_TURN_2 = 0.0;
 
  if(autoSelected == launch)
  {
    AUTO_DRIVE_SPEED = 0.0;
  }
  else if(autoSelected == drive)
  {
    AUTO_LAUNCHER_SPEED = 0.0;
  }
  else if(autoSelected == nothingAuto)
  {
    AUTO_DRIVE_SPEED = 0.0;
    AUTO_LAUNCHER_SPEED = 0.0;
  }
  else if(autoSelected == launchAndTurnRed)
  {
    AUTO_Z_TURN = 0.6;
    AUTO_Z_TURN_2 = 0.35;
  }
  else if(autoSelected == launchAndTurnBlue)
  {
    AUTO_Z_TURN = 0.2;
    AUTO_Z_TURN_2 = 0.2;
  }

  autonomousStartTime = Timer.getFPGATimestamp();

} // end of autonomousInit() method

/* ========================================== AUTO PERIODIC METHOD ===================================================== */

// start of autonomousPeriodic() method
@Override
public void autonomousPeriodic()
{

  double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

  // spins up launcher wheel until time spent in auto is greater than AUTO_LAUNCHER_DELAY_S
  if (timeElapsed < AUTO_LAUNCH_DELAY_S)
  {
    launchWheel.set(AUTO_LAUNCHER_SPEED);
    // DifferentialDrive method arcadeDrive(double xSpeed, double zRotation)
    drivetrain.arcadeDrive(0, 0);
  }
  // feeds note to launcher until time is greater than AUTO_DRIVE_DELAY_S
  else if (timeElapsed < AUTO_DRIVE_DELAY_S)
  {
    feedWheel.set(AUTO_LAUNCHER_SPEED);
    drivetrain.arcadeDrive(0, 0);
  }
  // drives until time is greater than AUTO_DRIVE_DELAY_S
  else if(timeElapsed < (AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S + 1.0) && AUTO_Z_TURN != 0)
  {
    launchWheel.set(0);
    feedWheel.set(0);
    if (timeElapsed < (AUTO_DRIVE_DELAY_S + AUTO_TURN_DELAY_S))
    {
      drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, AUTO_Z_TURN);
    }
    else
    {
      drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, AUTO_Z_TURN_2);
    }
  }
  else if(timeElapsed < (AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S) && AUTO_Z_TURN == 0)
  {
    launchWheel.set(0);
    feedWheel.set(0);
    drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, 0);
  }
  // does not move when time is greater than AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S
  else
  {
    if (AUTO_Z_TURN != 0)
    {
      if (timeElapsed < AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S + 1.0 + AUTO_TURN_2_DELAY_S)
      {
        drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, 1);
      }
      else
      {
        drivetrain.arcadeDrive(0,0);
      }
    }
    else
    {
      drivetrain.arcadeDrive(0, 0);
    }
  }

  // NEW AUTO CODE
  /* 
  if(autoSelected == nothingAuto)
  {
  }
  else if(autoSelected == launchAndDrive)
  {
    if(timeElapsed < AUTO_LAUNCH_DELAY_S)                                     // spin launch wheel (between 0-2 sec)
    {
      launchWheel.set(AUTO_LAUNCHER_SPEED);                                   // spins launch wheel
      drivetrain.arcadeDrive(0, 0);                          // 0 speed, 0 rotation drivetrain
    }
    else if(timeElapsed < AUTO_DRIVE_DELAY_S)                                 // spin feed wheel (between 2-3 sec)
    {
      feedWheel.set(AUTO_LAUNCHER_SPEED);                                     // spins feeder wheel (launch wheel still spinning)
      drivetrain.arcadeDrive(0, 0);                          // 0 speed, 0 rotation drivetrain
    }
    else if(timeElapsed < AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S)             // drive (between 3-5 sec)
    {
      launchWheel.set(0);                                               // turn off launch wheel
      feedWheel.set(0);                                                 // turn off feed wheel
      drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, 0);                  // drive at -50% speed (backward)
    }
    else                                                                      // remaining of the autonomous time
    {
      drivetrain.arcadeDrive(0, 0);                          // wait          
    }

  }
  else if(autoSelected == launch) 
  {
    
  }
  else if(autoSelected == drive)
  {

  }
  else if(autoSelected == launchAndFeed)
  {

  }
  */

} // end of autonomousPeriodic()

/* =============================================== TELEOP INIT METHOD ================================================== */

@Override
// start of teleopInit() method, called when teleop is enabled
public void teleopInit()
{

  // set all motors to coast mode
  /*
  leftRear.setIdleMode(IdleMode.kCoast);
  leftFront.setIdleMode(IdleMode.kCoast);
  rightRear.setIdleMode(IdleMode.kCoast);
  rightFront.setIdleMode(IdleMode.kCoast);
  */

  // set all motors to brake mode (in case it is preferred by driver)
  leftRear.setIdleMode(IdleMode.kBrake);
  leftFront.setIdleMode(IdleMode.kBrake);
  rightRear.setIdleMode(IdleMode.kBrake);
  rightFront.setIdleMode(IdleMode.kBrake);
  
} // end of teleopInit() method

/* ============================================== TELEOP PERIODIC METHOD =============================================== */

// create Timer object for shooting together method
Timer shooterTogetherTimer = new Timer();

// create Timer object for launcher in case want time the button is held down to not matter (consistently same time shooting)
Timer launcherTimer = new Timer();

// create Timer object for feeder in case want time the button is held down to not matter (consistently same time shooting)
Timer feederTimer = new Timer();

// create Timer object for climber
Timer climberTimer = new Timer();

@Override
// start of teleopPeriodic() method, called periodically during operator control
public void teleopPeriodic()
{

  // spins up the launcher wheel for the time that the button is held
  if(manipController.getRawButton(LAUNCHER_HOLD_OUT))
  {
    launchWheel.set(LAUNCHER_OUT_SPEED);
  }
  else if(manipController.getRawButtonReleased(LAUNCHER_HOLD_OUT))
  {
    launchWheel.set(0);
  }

  // spins up the launcher wheel for a set time after the button is pressed
  /* 
  if(manipController.getRawButton(LAUNCHER_TIME_OUT))
  {
    launcherTimer.restart();                              // timer starts
    if (LAUNCHER_TIME < launcherTimer.get())              // if preset time is less than current timer time
    {
      launchWheel.set(LAUNCHER_SPEED);                    // turn on
    }
    else
    {
      launchWheel.set(0);                                 // turn off
    }
    launcherTimer.reset();
    launcherTimer.stop();                                 // stop timer
  }
  */
  
  // spins feeder wheel out
  if(manipController.getRawButton(FEEDER_HOLD_OUT))
  {
    feedWheel.set(FEEDER_OUT_SPEED);
  }
  else if(manipController.getRawButtonReleased(FEEDER_HOLD_OUT))
  {
    feedWheel.set(0);
  }

  // spins feeder wheel in
  /*
  if(manipController.getRawButton(FEEDER_HOLD_IN))  
  {
    feedWheel.set(FEEDER_IN_SPEED);
  }
  else if(manipController.getRawButtonReleased(FEEDER_HOLD_IN))
  {
    feedWheel.set(0);
  }
  */

  // spins up the feeder wheel for a set time after the button is pressed
  /*
  if(manipController.getRawButton(FEEDER_TIME_OUT))
  {
    feederTimer.restart();
    if (FEEDER_TIME <= feederTimer.get())
    {
      feedWheel.set(FEEDER_OUT_SPEED);
    }
    else
    {
      feedWheel.set(0);
    }
    feederTimer.reset();
    feederTimer.stop();
  }
  */
  
  // both motors spun to intake note
  if(manipController.getRawButton(BOTH_MOTORS_SHOOTER_IN))
  {
    launchWheel.set(LAUNCHER_IN_SPEED);
    feedWheel.set(FEEDER_IN_SPEED);
  }
  else if(manipController.getRawButtonReleased(BOTH_MOTORS_SHOOTER_IN))
  {
    launchWheel.set(0);
    feedWheel.set(0);
  }
  
  // spin both motors to "spit" the note out at a lower speed into the amp
  /*
  if(manipController.getRawButton(SHOOTER_AMP_HOLD_OUT))
  {
    feedWheel.set(FEEDER_AMP_SPEED);
    launchWheel.set(LAUNCHER_AMP_SPEED);
  }
  else if(manipController.getRawButtonReleased(SHOOTER_AMP_HOLD_OUT))
  {
    feedWheel.set(0);
    launchWheel.set(0);
  }
  */

  // special button for full shooting mechanism
  // values for time need changing on line 121/122
  /*
  if (manipController.getRawButton(FULL_SHOOTING_MECHANISM))
  {
    launchWheel.set(LAUNCHER_SPEED);
    // try the timer method which may throw an exception if the timer fails or the system interrupts the timer
    // catch that exception and do it the old fashioned way
    try 
    { 
      shooterTogetherTimer.wait((long) (BOTH_SHOOTER_LAUNCHER_DELAY * 1000));
    } catch (InterruptedException e) {
      shooterTogetherTimer.restart();
      while(shooterTogetherTimer.get() <= BOTH_SHOOTER_END_TIME)
      {
        launchWheel.set(LAUNCHER_SPEED);
        if (shooterTogetherTimer.get() <= BOTH_SHOOTER_LAUNCHER_DELAY)
        {
          launchWheel.set(LAUNCHER_SPEED);
          feedWheel.set(LAUNCHER_SPEED);
        }
      }
      launchWheel.set(0);
      feedWheel.set(0);
      shooterTogetherTimer.reset();
      shooterTogetherTimer.stop();
    }
    feedWheel.set(LAUNCHER_SPEED);
  }
  */

  // X button: spin launch wheel and then after a time spin feed wheel
  if(manipController.getRawButton(FULL_SHOOTING_MECHANISM))
  {
    shooterTogetherTimer.restart();
    while(shooterTogetherTimer.get() <= BOTH_SHOOTER_END_TIME)
    {
      launchWheel.set(LAUNCHER_SPEED);
      if (shooterTogetherTimer.get() >= BOTH_SHOOTER_LAUNCHER_DELAY)
      {
        feedWheel.set(LAUNCHER_SPEED);
      }
    }
    launchWheel.set(0);
    feedWheel.set(0);
    shooterTogetherTimer.reset();
    shooterTogetherTimer.stop();
  } 

  // on for time button
  if (manipController.getRawButton(ON_LONG_BUTTON))
  {
    launchWheel.set(LAUNCHER_OUT_SPEED);
  }


  // on and off button draft #1
   /* 
  if(manipController.getRawButton(ON_LONG_BUTTON))
  {
    if (TOP_MOTOR_RUNNING == true)
    {
      launchWheel.set(0);
      TOP_MOTOR_RUNNING = false;
    }
    else
    {
      TOP_MOTOR_RUNNING = true;
    }

    while (TOP_MOTOR_RUNNING)
    {
      launchWheel.set(LAUNCHER_OUT_SPEED);
    }
  }
  */

  // off button (just in case)
  /*
  if (manipController.getRawButton(OFF_LONG_BUTTON))
  {
    launchWheel.set(0);
  }
  */
  
  // spin roller claw
  if(manipController.getRawButton(ROLLER_CLAW_IN))
  {
    rollerClaw.set(CLAW_OUTPUT_POWER);                         // positive output power
  }
  else if(manipController.getRawButton(ROLLER_CLAW_OUT))
  {
    rollerClaw.set(-CLAW_OUTPUT_POWER);                        // negative output power
  }
  else
  {
    rollerClaw.set(0);                                   // turns roller claw off if a button is not pressed
  }

  // keep roller claw on passively
  /* 
  if(manipController.getRawButton(ROLLER_CLAW_PASSIVE_OUT))
  {
    rollerClaw.set(-CLAW_OUTPUT_POWER);
  }
  else
  {
    rollerClaw.set(CLAW_OUTPUT_POWER);
  }
  */

  // climber (POV is D-PAD; 0 == Up and 180 == Down)
  if(manipController.getPOV() == 0)
  {
    climber.set(1);
  }
  else if(manipController.getPOV() == 180)
  {
    climber.set(-1);
  }
  else
  {
    climber.set(0);
  }

  // climber with button code
  /*
  if(manipController.getRawButton(CLIMBER_UP))
  {
    climber.set(1);                                                   // climber up
  }
  else if(manipController.getRawButton(CLIMBER_DOWN))              
  {
    climber.set(-1);                                                 // climber down
  }
  else
  {
    climber.set(0);
  }
  */
  
  
  // climber all up
  /*
  if(manipController.getRawButton(CLIMBER_ALL_UP))
  {
    climberTimer.restart();
    while(climberTimer.get() <= FULL_CLIMBER_SECONDS)
    {
      climber.set(1);
      try {
        climberTimer.wait((long) (FULL_CLIMBER_SECONDS * 1000));
      } catch (InterruptedException exception) {
        // ignore and continue in loop
      }
    }
    climberTimer.reset();
    climberTimer.stop();
  }
  */

  // climber partial up
  /*
  if(manipController.getRawButton(CLIMBER_PARTIAL_UP))
  {
    climberTimer.restart();
    while(climberTimer.get() <= PARTIAL_CLIMBER_SECONDS)
    {
      climber.set(1);
      try {
        climberTimer.wait((long) (PARTIAL_CLIMBER_SECONDS * 1000));
      } catch (InterruptedException exception) {
        // ignore and continue in loop
      }
    }
    climberTimer.reset();
    climberTimer.stop();
  }
  */

  // DifferentialDrive method: arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) 
  drivetrain.arcadeDrive(-driverController.getRawAxis(1), -driverController.getRawAxis(0), false);

  // slow button for drive train: CHANGE TO 50%
  // multiplied by 0.25 for 25%; multiplied by 0.5 for 50%
  
  if(driverController.getRawButton(2))
  {
    drivetrain.arcadeDrive(-driverController.getRawAxis(1) * 0.5, -driverController.getRawAxis(0) * 0.5, false);
  }
  else
  {
    drivetrain.arcadeDrive(-driverController.getRawAxis(1), -driverController.getRawAxis(0), false);
  }
  

  // With less sensitivity:
  /*
  // DifferentialDrive method: arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) (with less sensitivity)
  drivetrain.arcadeDrive(-driverController.getRawAxis(1), -Math.pow(driverController.getRawAxis(0), 3), false);

  // slow button for drive train (with less sensitivity)
  if(driverController.getRawButton(2))
  {
    drivetrain.arcadeDrive(-driverController.getRawAxis(1) * 0.25, -Math.pow(driverController.getRawAxis(0), 3) * 0.25, false);
  }
  else
  {
    drivetrain.arcadeDrive(-driverController.getRawAxis(1), -Math.pow(driverController.getRawAxis(0), 3), false);
  }
*/

} // end of teleopPeriodic() method

} // end of Robot class