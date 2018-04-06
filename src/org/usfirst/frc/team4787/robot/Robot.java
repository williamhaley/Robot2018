//2018
package org.usfirst.frc.team4787.robot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

/*
while(RobotOn){
	Robot.works();
}
//returns error "Kevin is dumb"
 * 
 * public static void teleop(String[] args)
 * if (Joystick1.pressed)
 * Robot.PutsCubeOnSwitch;
 * *if (Joystick2.pressed)
 * Robot.PutsCubeOnScale;
** if (Joystick3.pressed)
 * Robot.PutsCubeInVault;
 * if (Joystick4.pressed)
 * Robot.Climbs;
 * if (Robot = Off)
 * 	Robot.DoesntWork;

**/

public class Robot extends SampleRobot {
	// min value in x and y direction for robot to respond to joystick movements
	final double DEADZONEX = 0.01, DEADZONEY = 0.01;
	
    // button configuration
	final int Pnematic_Open = 7;	//stick 2
	final int SlowHold = 8;			//stick 1
	final int Lift_Intake = 7; 		//stick 1 

	//Secondary *not used*
	final int DISABLE_LIFT = 12;	//stick 2
	final int Reduced_Drive = 8;	//stick 2
	final int FlySpeedLaunch = 9;	//stick 2 
	final int FlySpeedPlace = 10;	//stick 2

	//motor speeds 
	double flywheelSpeed = 0.4;
    double flywheelSpeedLaunch = 1;
    double flywheelSpeedIntake = 0.7;
	double Kp = 0.03;

	//boolean for certain mechanisms
    boolean stopFlyWheels = true;
	boolean speedReduced = false;
	boolean pneumaticOpenClose = false;
	boolean launchCube = true;
	
	//joystick instantiation
    Joystick stick = new Joystick(2);
    Joystick stick2 = new Joystick(0);
    
    //limit switch
    DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    double output;
    
    //motor variable instantiation// Actual instantiation happens in Robot.ini
    	Spark forkLiftMech;
    	Spark fly1;
    	Spark fly2;
    	Talon back_Left;
    	Talon front_Left;
    	SpeedControllerGroup m_Left;
    	Talon back_Right;
    	Talon front_Right;
    	SpeedControllerGroup m_Right;
    	//DifferentialDrive m_drive;
    
    //pneumatics instantiation
    DoubleSolenoid sol1;
    Compressor c;
    //c.setClosedLoopControl(true);
    //sets up the compressor setting/status
    boolean enabled;
    boolean pressureSwitch;
    double current;
    UsbCamera cam0;
    
    AnalogGyro gyro;
    AnalogAccelerometer accel;

    // sets RobotDrive obj to null so that auto code works
    DifferentialDrive m_drive = null;
    
    // variable to handle initialization of Robot Drive obj//i.e., on or off?
    boolean initRobotDrive;
    
    // String variable to determine auto mode, "L" = left starting position, "M" = middle starting position, "R" = right starting posistion
    String autoMode;
    
    // various variables to control robot mechanisms and drive train
    double y, x, expY, expX, autoPower, autoTime, leftDoorAngleStart, rightDoorAngleStart, climbSpeed;
    
    /**
     * Constructs Robot object with necessary configurations
     */
    
    @Override
    public void robotInit(){
    	UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
        cam0.setResolution(320, 240);
        cam0.setFPS(15);
        // limit switch
        forwardLimitSwitch = new DigitalInput(1);
        
        //Left Motors
         back_Left = new Talon(2);
         front_Left = new Talon(3);
         m_Left = new SpeedControllerGroup(front_Left, back_Left);
        //Right Motors
         back_Right= new Talon(1);
         front_Right = new Talon(0);
         m_Right = new SpeedControllerGroup(front_Right, back_Right);
        //Instantiate Differential Drive
         m_drive = new DifferentialDrive(m_Left, m_Right);
        
        //Other motors 
         forkLiftMech = new Spark(6);
         fly1 = new Spark(4); //left 
         fly2 = new Spark(5); //right
        
        //parameter is nodeid of the solenoids & compressor 
         sol1 = new DoubleSolenoid(4,5);
         c = new Compressor(0);
        //c.setClosedLoopControl(true);
        //sets up the compressor setting/status
         enabled = c.enabled();
         pressureSwitch = c.getPressureSwitchValue();
         current = c.getCompressorCurrent();
         //Gyro initilization code
         gyro = new AnalogGyro(0);
         //accel = new AnalogAccelerometer(0);
         gyro.initGyro();
         gyro.calibrate();
    	
    	
    }
    public Robot() {
    	
    	initRobotDrive = true;
//        UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);
//        cam0.setResolution(320, 240);
//        cam0.setFPS(15);
    	
    	climbSpeed = 0;

	    JoystickButton pneumaticOpen = new JoystickButton(stick2, Pnematic_Open);
	    JoystickButton reducedDrive = new JoystickButton(stick2, Reduced_Drive);
	    JoystickButton flyLaunch = new JoystickButton(stick2, FlySpeedLaunch);
	    JoystickButton flyPlace = new JoystickButton(stick2, FlySpeedPlace);
	    
	    flyLaunch.whenPressed(new InstantCommand(){
	    	@Override
	    	protected void execute() {
	    		launchCube = true;
	    		
			}
	    });
	    
	    flyPlace.whenPressed(new InstantCommand(){
	    	@Override
	    	protected void execute() {
	    		launchCube = false;
	    		
			}
	    });
	    
	    reducedDrive.whenPressed(new InstantCommand(){
	    	@Override
			protected void execute() {
				if(speedReduced == false){
		    		System.out.println("Reduced Drive ON");
		    		speedReduced = true;
				}
				else if(speedReduced == true){
		    		System.out.println("Reduced Drive OFF");
		    		speedReduced = false;
				}
			}
	    });

        pneumaticOpen.whenPressed(new InstantCommand(){
	    	@Override
	    	protected void execute() {
	    		System.out.println("pneumatic open");
	    		if(pneumaticOpenClose == false){
	    			sol1.set(DoubleSolenoid.Value.kForward);
	    			pneumaticOpenClose = true;
	    		}
	    		else{
	    			sol1.set(DoubleSolenoid.Value.kReverse);
	    			pneumaticOpenClose = false;
	    		}
			}
	    });
	    
//	    JoystickButton pneumaticOpen= new JoystickButton(stick, 9);
//	    JoystickButton pneumaticClose= new JoystickButton(stick, 10);
//	    pneumaticOpen.whenPressed(new InstantCommand(){
//	    	@Override
//			protected void execute() {
//				System.out.println("pneumatic open");
//		    	sol1.set(DoubleSolenoid.Value.kForward);
//		    	//sol1.set(DoubleSolenoid.Value.kOf false);
//			}
//	    });
//	    pneumaticClose.whenPressed(new InstantCommand(){
//	    	@Override
//			protected void execute() {
//				System.out.println("pneumatic close");
//		    	sol1.set(DoubleSolenoid.Value.kReverse);
//		    	//sol1.set(DoubleSolenoid.Value.kOf false);
//			}
//	    });


    }//end of public robot constructor 
    
    
    
    
    
    public void autoPause(){
    	back_Left.stopMotor();
		front_Left.stopMotor();
		back_Right.stopMotor();
		front_Right.stopMotor();
    }
    
    public void drive(double speed, double time, boolean isReverse) {
    	
    	if(speedReduced){
    		speed = 0.5*speed;
    	}
		speed = isReverse ? -speed : speed;
		back_Left.set(speed);
		front_Left.set(speed);
		back_Right.set(-speed);
		front_Right.set(-speed);
		Timer.delay(time);
		this.autoPause();
		
		//ok this one is also moving forward, but you at least have some factors that tell the robot how to move as a parameter
    }
    
    
    /**
     * Method for robot to do a 180 degree turn in auto
     */
    public void turnAround() {
    	autoPower = 0.2;
		autoTime = 0.7;
		back_Left.set(autoPower);
		front_Left.set(autoPower);
		back_Right.set(autoPower);
		front_Right.set(autoPower);
		Timer.delay(autoTime);
		this.autoPause();
		//a really long left turn
		//moves the robot around 180 degrees
    }
    
    /**
     * Method for robot to turn left in auto
     */
    public void turnLeft() {
    	autoPower = 0.2;
		autoTime = 0.35;
		back_Left.set(autoPower);
		front_Left.set(autoPower);
		back_Right.set(autoPower);
		front_Right.set(autoPower);
		Timer.delay(autoTime);
		this.autoPause();
		//left turn
    }
    
    /**
     * Method for robot to turn right in auto
     */
    public void turnRight() {
    	autoPower = 0.2;
		autoTime = 0.33;
		back_Left.set(-autoPower);
		front_Left.set(-autoPower);
		back_Right.set(-autoPower);
		front_Right.set(-autoPower);
		Timer.delay(autoTime);
		this.autoPause();
		//opposite of left turn, aka a right turn
    }
    
    public void dropBoxDown(){
		autoTime = 0.1;
    	fly1.set(flywheelSpeed);
		fly2.set(flywheelSpeed);
		Timer.delay(autoTime);
		fly1.set(0);
		fly2.set(0);
    }
    
    public void liftUp(){
    	autoTime = 0.1;
    	forkLiftMech.set(climbSpeed);	
		Timer.delay(autoTime);
		forkLiftMech.stopMotor();
    }
    
    public void stopLift(){
    	forkLiftMech.stopMotor();
    }
    
    public void cleanSet(){
    	this.dropBoxDown();
    	this.liftUp();
    	this.stopLift();
		this.drive(.2, 6,  true);
    }
    
    public void autoSwitch(){
    	
    }
    
    //commands integrated with gyro, meant to be run in a loop
    public void gyroDriveStraight() {
    	double angle = gyro.getAngle(); // get current heading
    	m_drive.arcadeDrive(0.2, -angle*Kp); // drive towards heading 0
    }
    public void gyroTurnLeft(){
        double angle = gyro.getAngle(); // get current heading
        double difference = angle - 90;
        m_drive.arcadeDrive(0.2, difference*Kp); // drive towards heading 0
    }
    
    //facingLeft checker meant to be run in a loop
    public boolean facingLeft(){
    	if(gyro.getAngle()>=-89 && gyro.getAngle() <= -91)
    		return true;
    	else
    		return false;
    }
    
    public void gyroTurnRight(){
    	double angle = gyro.getAngle(); // get current heading
        double difference = angle + 90;
        m_drive.arcadeDrive(0.2, difference*Kp); // drive towards heading 0
    }
    
    public boolean facingRight(){
    	if(gyro.getAngle()>= 89 && gyro.getAngle() <= 91)
    		return true;
    	else
    		return false;
    }
    
    public void gyroTurnAround(){
    	double angle = gyro.getAngle(); // get current heading
        double difference = angle + 180;
        m_drive.arcadeDrive(0.2, difference*Kp); // drive towards heading 0
    }
    
    public boolean facingAround(){
    	if(gyro.getAngle()>= 179 && gyro.getAngle() <= 181)
    		return true;
    	else
    		return false;
    }
    
    public void leftSideSwitchAuto(){
    	int step = 1;
    	long startTime = System.currentTimeMillis();
    	 gyro.reset();
         while (isAutonomous()) {
        	 if((step == 1 && System.currentTimeMillis()-startTime >= 6500)){//time to move forward from base 
        		 step = 2;
        	 }
        	 if(step == 2 && facingLeft()){
        		 step = 3;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 3 && System.currentTimeMillis()-startTime >= 5000){
        		 step = 4;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 4 && facingRight()){
        		 step = 5;
        	 }
        	 if(step == 5 && System.currentTimeMillis()-startTime >= 1000){
        		 step = 6;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 6 && facingAround()){
        		 step = 7;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 7 && System.currentTimeMillis()-startTime >= 1000){
        		 step = 8;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 8 && System.currentTimeMillis()-startTime >= 500){
        		 step = 9;
        		 startTime = System.currentTimeMillis();

        	 }
        	 if(step == 9 && System.currentTimeMillis()-startTime >= 1500){
        		 step = 10;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 10 && System.currentTimeMillis()-startTime >= 1500){
        		 step = 11; 
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 11 && facingAround()){
        		 step = 12; 
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 12 && System.currentTimeMillis()-startTime >= 3000){
        		 step = 13;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 13 && System.currentTimeMillis()-startTime >= 1000){
        		 step = 14;
        	 }
        	 
        	 
        	 
        	 
        	 ///Steps that occur when conditions are met
        	 if(step == 1){     		 
                 gyroDriveStraight();
        	 }
        	 if(step == 2){
        		 gyroTurnLeft(); 
        	 }
        	 if(step == 3){
        		 gyro.reset();
                 gyroDriveStraight();
        	 }
        	 if(step == 4){
        		 gyroTurnRight();
        		 Timer.delay(1);
        	 }
        	 if(step == 5){ 
        		 sol1.set(DoubleSolenoid.Value.kForward);
        		 fly1.set(0.25);
        		 fly2.set(0.25);
                 Timer.delay(0.5);
                 fly1.set(0);
        		 fly2.set(0);
        	 }
        	 if(step == 6){
        		 gyro.reset();
        		 gyroTurnAround();
        		 fly1.set(-0.5);
        		 fly2.set(-0.5);
        	 }
        	 if(step == 7){
        		 gyro.reset();
                 gyroDriveStraight();
        	 }
        	 if(step == 8){
        		 sol1.set(DoubleSolenoid.Value.kReverse);
        		
        	 }
        	 if(step == 9){
        		 gyro.reset();
                 gyroDriveStraight();
        		
        	 }
        	 if(step == 10){
        		 fly1.set(1);
        		 fly2.set(1);
        	 }
        	 if(step == 11){
        		 gyroTurnAround();
        	 }
        	 if(step == 12){
        		 gyro.reset();
        		 sol1.set(DoubleSolenoid.Value.kForward);
        		 fly1.set(-0.5);
        		 fly2.set(-0.5);
                 gyroDriveStraight();
        	 }
        	 if(step == 13){
        		 fly1.set(-0.18);
        		 fly2.set(-0.18);
        		 gyroTurnAround();
        		 sol1.set(DoubleSolenoid.Value.kReverse);
        		 gyro.reset();
                 gyroDriveStraight();

        	 }
        	 if(step == 14){
        		 fly1.set(1);
        		 fly2.set(1);
        		 
        	 }
             Timer.delay(0.004);
         }
         this.autoPause(); 
        	 
         }
    	
    
    public void rightSideSwitchAuto(){
    	int step = 1;
    	long startTime = System.currentTimeMillis();
    	 gyro.reset();
         while (isAutonomous()) {
        	 ///Conditions that must be met to move on to the next step 
        	 if((step == 1 && System.currentTimeMillis()-startTime >= 5000)){
        		 step = 2;
        	 }
        	 if(step == 2 && facingLeft()){
        		 step = 3;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 3 && System.currentTimeMillis()-startTime >= 1000){
        		 step = 4;
        	 }
        	 //step 4 to 5 is skipped as it is put in the runtime if statement
        	 if(step == 5 && facingRight()){
        		 step = 6;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 6 && System.currentTimeMillis()-startTime >= 2000){
        		 step = 7;
        	 }
        	 if(step == 7 && facingLeft()){
        		 step = 8;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 8 && System.currentTimeMillis()-startTime >= 1000){
        		 step = 9;
        	 }
        	 if(step == 9 && facingLeft()){
        		 step = 10;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 10 && System.currentTimeMillis()-startTime >= 1500){
        		 step = 11; 
        	 }
        	 //step == 11 skipped, put in runtime if
        	 if(step == 12 && System.currentTimeMillis()-startTime >= 1000){
        		 step = 13;
        	 }
        	 
        	 
        	 
        	 
        	 
        	 ///Steps that occur when conditions are met
        	 if(step == 1){     		 
                 gyroDriveStraight();
        	 }
        	 if(step == 2){
        		 gyroTurnLeft(); 
        	 }
        	 if(step == 3){
        		 gyro.reset();
                 gyroDriveStraight();
        	 }
        	 if(step == 4){
        		 fly1.set(1);
        		 fly2.set(1);
                 Timer.delay(1);
                 fly1.set(-.18);
        		 fly2.set(-.18);
        		 step = 5;
        	 }
        	 if(step == 5){
        		 gyroTurnRight();
        	 }
        	 if(step == 6){
        		 gyro.reset();
                 gyroDriveStraight();
        	 }
        	 if(step == 7){
        		 gyroTurnLeft();
        	 }
        	 if(step == 8){
        		 gyro.reset();
                 gyroDriveStraight();
        	 }
        	 if(step == 9){
        		 gyroTurnLeft();
        		 sol1.set(DoubleSolenoid.Value.kForward);
        	 }
        	 if(step == 10){
        		 fly1.set(-.4);
        		 fly2.set(-.4);
        		 gyro.reset();
                 gyroDriveStraight();
                 
        	 }
        	 if(step == 11){
        		 sol1.set(DoubleSolenoid.Value.kReverse);
        		 Timer.delay(1.5);
        		 fly1.set(1);
        		 fly2.set(1);
        		 Timer.delay(1);
                 fly1.set(-.18);
        		 fly2.set(-.18);
        		 step = 12;
        		 startTime = System.currentTimeMillis();
        	 }
        	 if(step == 12){
        		 sol1.set(DoubleSolenoid.Value.kForward);
        		 double angle = gyro.getAngle(); // get current heading
        		 m_drive.arcadeDrive(-0.2, -angle*Kp); // drive towards heading 0
        	 }
        	 if(step == 13){
        		 m_drive.arcadeDrive(0,0); // pause drive
        	 }
             Timer.delay(0.004);
         }
         this.autoPause();
      }
    	
    
    
    
    
    
    /**
     * AUTONOMOUS
     */
    public void autonomous() {
    	/*String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(gameData.charAt(0) == 'L'){
			//this.leftSideSwitchAuto();
			this.drive(0.2, 5.05, false);

		}
		if(gameData.charAt(0) == 'R'){
			//this.rightSideSwitchAuto();
			//this.liftUp();
	    	//this.stopLift();
			this.drive(0.2, 5.05, false);
			//sol1.set(DoubleSolenoid.Value.kReverse);
   		 	//Timer.delay(1.5);
   		 	//fly1.set(1);
   		 	//fly2.set(1);
   		 	//Timer.delay(1);
            //fly1.set(-.18);
   		 	//fly2.set(-.18);

		}
		else{
			this.drive(0.2, 5.05, false);
		}

    
    	//this.drive(0.2, 5.05, false);
    */	
    	this.drive(0.6, 5.05, false);
      }
    	

		//this.drive(0.5, 1,  false);		
		
		
    	/* 
    	int switchOrScale = 3;
    	//Switch = 0, Scale = 1; // Override = 3;
    	int LMR = 0;
    	//Left = 0, Middle = 1, Right = 2
    	String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (switchOrScale == 0 ){
			//The robot is going for the scale
			if(gameData.length() > 0)
	        {
			 if(gameData.charAt(0) == 'L')
			  {
			
	         }
		}
		else if (switchOrScale == 1){
			//NOTE: THE SAME CODE FOR SCALE IS USED HERE. The main difference between the two will be the DISTANCES traveled by the robot.
			if(gameData.length() > 0)
	        {
			 i
		else if (switchOrScale == 3){
			this.drive(.3, 10,  false);
			

		}    	
    		
    	
    
    /**
     * TELEOPERATED MODE. Runs the motors with arcade steering.
     */
    public void operatorControl() {
    	// code to initialize RobotDrive in tele op mode
    	
   
    	if(initRobotDrive || m_drive.equals(null)) {
//        	arcadeDrive = new RobotDrive(front_Left, back_Left, front_Right, back_Right);
//        	arcadeDrive.setSafetyEnabled(false);
    		m_drive = new DifferentialDrive(m_Left, m_Right);
        	System.out.println("robotDrive initialize");
        	//arcadeDrive = new RobotDrive(fLeft, bLeft, fRight, bRight);
        	initRobotDrive = false;
        }
    	
    	// teleop code loop
    	while (isOperatorControl() && isEnabled()) {
    		//programs motor controllers to drive with exponential arcade drive (i.e. real values are dampened by exponentiation to make driving smoother)
        	boolean slow = stick.getRawButton(SlowHold);
    		if (!slow){
    			expY = (Math.pow(-stick.getY(), 1))/1.8;
        		expX = (Math.pow(-stick.getX(), 1))/1.8;
    		} else if (slow) {
    			expY = Math.pow(-stick.getY(), 1);
    			expX = Math.pow(-stick.getX(), 1);
    		}
    		
    		//arcadeDrive.arcadeDrive(expY, expX);
    		
    	   	m_drive.arcadeDrive(expY,-expX,true);
    		
             expX = 0;
             expY = 0;
             if (Math.abs(x) > DEADZONEX) {
                 expX = x * Math.abs(x);
             } else {
                 expX = 0;
             }
             if (Math.abs(y) > DEADZONEY) {
                 expY = y;
             } else {
                 expY = 0;
             }

    	    		
        	boolean disableLiftMotor = stick2.getRawButton(DISABLE_LIFT);
        	boolean flywheelsOn = stick.getTrigger();
        	boolean flywheelsIn = stick.getRawButton(Lift_Intake);
        
        	
        	// determines whether the POV hat switch is set to correct state to trigger events for climbing
        	boolean accelerateforkLiftMech = stick2.getY() <= 0.01 ? true : false;
        	boolean decceraateforkLiftMech = stick2.getY() >= 0.01 ? true : false;
        	
        	output = stick2.getY();
    		// if the DISABLE_LIFT button is pressed, stop the climbing mech motor
        	if (disableLiftMotor) {
        		forkLiftMech.stopMotor();
        	}
//        	if (forwardLimitSwitch.get()){
//        		output = Math.min(output, 0);
//        	}
        		
        	
        	// if the POV stick (hat switch) is moved to the forward position, accelerate the climbing mech motor
        	if (accelerateforkLiftMech) {
//			    climbSpeed = 0.6;
        		forkLiftMech.set(output); //don't know what to set this to
		    	}
        	
        	// if the POV stick (hat switch) is moved to the forward position, decelerate the climbing mech motor
        	if (decceraateforkLiftMech) {
//        		climbSpeed = -0.4;
        		forkLiftMech.set(output); //don't know what to set this to

        	}
        	
        	if(flywheelsOn && launchCube) {
        		fly1.set(flywheelSpeedLaunch);
        		fly2.set(flywheelSpeedLaunch);
        	}else if (flywheelsOn && !launchCube) {
        		fly1.set(flywheelSpeed);
        		fly2.set(flywheelSpeed);
        	}else if (flywheelsIn) {
        		fly1.set(-flywheelSpeedLaunch);
        		fly2.set(-flywheelSpeedLaunch);
        	}else if (stopFlyWheels){
        		fly1.set(-0.5);
        		fly2.set(-0.5);
        	}
        	
        	
        	
        
        	
//        	if (flywheelsOn == false && flywheelsIn == false) {
//        		fly1.stopMotor();
//        		fly2.stopMotor();
//        	}
        	
        	
        	Scheduler.getInstance().run();
    	    Timer.delay(0.0005);		// wait for a motor update time
        }
    }

    

    /**
     * Runs during test mode
     */
    
 
    //reference: https://github.com/chopshop-166/frc-2017/blob/master/src/org/usfirst/frc/team166/robot/Robot.java
    
  /*  public void selectauto(){
    	switch(selected){
	    	case "Autonomous 1":
	    		this.auto1;
	    		break;
	    	case "Autonomous 2":
	    		this.auto2;
	    		break;
	    	case "Autonomous 3":
	    		this.auto3;
	    		break;
	    	case "Autonomous 4":
	    		this.auto4;
	    		break;
	    	case "Autonomous 5":
	    		this.auto5;
	    		break;
	    	case "Autonomous 6":
	    		this.auto6;
	    		break;
	    	default:
	    		this.auto1;
	    		break;
    	}
    	}
    */
    
    
    /**
     * Method to stop drive train motors
     */
    public void stopMotors() {
    	back_Left.set(0);
		front_Left.set(0);
		back_Right.set(0);
		front_Right.set(0);
    }
    
    /**
     * Runs while robot is disabled
     */
    public void disabled()
    {
    	// disables all motors and servos on the robot
    	this.stopMotors();
    	fly1.set(0);
    	fly2.set(0);
    	sol1.set(DoubleSolenoid.Value.kReverse);
    	forkLiftMech.set(0);
    	System.out.println("I prefer differently abled you ableist");
    }
}
