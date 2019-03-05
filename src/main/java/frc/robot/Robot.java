package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.cscore.UsbCamera;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.GenericHID;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	final String drivecontroller = "Drive Controller";
	final String drivejoystick = "Drive Joystick";
	boolean driveControllerSelected = false;
	//SendableChooser<String> drive = new SendableChooser<>();

	final String buttonscontroller = "Buttons Controller";
	final String buttonsjoystick = "Buttons Joystick";
	boolean buttonsControllerSelected = false;
	//SendableChooser<String> buttons = new SendableChooser<>();

	WPI_TalonSRX frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
	Joystick driveJoystick, buttonsJoystick, driveController, buttonsController;
	HHJoystickButtons driveJoystickButtons, buttonsJoystickButtons;
	MecanumDrive driveTrain;
	
	Spark liftA, liftB;
	SpeedController lift;
	VictorSPX intake;
	//Spark intake;

	UsbCamera usbCamera;
	UsbCamera usbCamera2;
	
	//DoubleSolenoid actuator1, actuator2;
	
	static boolean SHOOT_BUTTON = false;//1;
	static boolean BALL1_BUTTON = false;//4;
	static boolean BALL2_BUTTON = false;//5;
	static boolean BALL3_BUTTON = false;//6;
	static boolean PLATE1_BUTTON = false;//7;
	static boolean PLATE2_BUTTON = false;//8;
	static boolean PLATE3_BUTTON = false;//9;
	static boolean BALL_CS_BUTTON = false;//10;
	static boolean BALLINTAKE_BUTTON = false;//1;
	static boolean BALLINTAKE_REVERSE_BUTTON_PUSHED = false;//2;
	static boolean BALLINTAKE_REVERSE_BUTTON_RELEASED = false;
	static boolean CLIMB_BUTTON = false;
	static final int BALL1_HEIGHT = 1581;
	static final int BALL2_HEIGHT = 3664;
	static final int BALL3_HEIGHT = 5170;
	static final int PLATE1_HEIGHT = 253;
	static final int PLATE2_HEIGHT = 2184;
	static final int PLATE3_HEIGHT = 4320;
	static final int BALL_CS_HEIGHT = 1711;
	static final int forwardChannel1 = 1;
	static final int reverseChannel1 = 0;
	static final int forwardChannel2 = 5;
	static final int reverseChannel2 = 4;
	static final int forwardChannel3 = 2;
	static final int reverseChannel3 = 3;
	static final int forwardChannel4 = 6;
	static final int reverseChannel4 = 7;
	static final DoubleSolenoid.Value RETRACT = DoubleSolenoid.Value.kForward;
	static final DoubleSolenoid.Value PUSH = DoubleSolenoid.Value.kReverse;
	static final Double driveDeadpan = 0.2;

	static final double liftSpeed = -0.8;
	static final double intakeSpeed = -0.6;

	double climbReverseSpeed = -1.0;
	double climbReverseTime = 1.0;
	double climbForwardSpeed = 1.0;
	double climbForwardTime = 1.0;
	double delayAfterClimbPistonsOut = 1.0;
	double climbForward2Speed = 1.0;
	double climbForward2Time = 1.0;
	double delayAfterClimbPistonsRetract = 1.0;

	double delayAfterWheelIntakeOut = 0.2;
	double delayBeforeRetract = 1;

	double CURRENT_LIMIT = 10;

	boolean liftControl = false;

	//boolean controllers;

	/*double joystickB;
	double joystickA;
	double joystickY;
	double joystickZ;*/

	boolean isWheelsOut;

	double targetTimeW = 0;
	double targetTimeL = 0;
	double targetTimeC = 0;
	
	AHRS ahrs;

	PowerDistributionPanel pdp;

	String button;
	String piston;
	String buttonOne;
	int distance;

	static enum MotorState{
		Stopped, Forward, Reverse, Other
	} 

	MotorState liftMotor = MotorState.Stopped;
	static enum LiftState{
		Bottom, Rising, FootFromTop, Top, Shoot, Descending, FootFromBottom, control
	}

	static enum WheelIntakeCase{
		wheelIntakeStowed, wheelIntakeOffAndOut, wheelIntakeOnAndOut, wheelIntakeOffAndOut2, wheelIntakeReverse
	}

	static enum ClimbState{
		stopped, reverse, forward, lift, forward2, pistonsRetract, finalDriveForward
	}

	WheelIntakeCase wheelIntakeCase;
	
	ClimbState climbState;

	LiftState liftState;
	double height = 0;
	boolean isBall;

	Encoder enc;

	double Current = 0;

	DoubleSolenoid ballPusher, platePusher, /*platePusher2,*/ pistonW, climbPistons; //pistonW2;


	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		enc = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		enc.reset();

		

		/*drive.setDefaultOption("Drive Controller", drivecontroller);
		drive.addOption("Drive Joystick", drivejoystick);
		SmartDashboard.putData("Drive choices", drive);
		driveControlsSelected = drive.getSelected();

		buttons.setDefaultOption("Buttons Controller", buttonscontroller);
		buttons.addOption("Buttons Joystick", buttonsjoystick);
		SmartDashboard.putData("Buttons choices", buttons);
		buttonsControlsSelected = buttons.getSelected();*/
		
		frontLeftMotor = new WPI_TalonSRX(2);
		rearLeftMotor = new WPI_TalonSRX(0);
		frontRightMotor = new WPI_TalonSRX(3);
		rearRightMotor = new WPI_TalonSRX(1);

		rearLeftMotor.set(0.7);
		rearRightMotor.set(0.9);

		intake = new VictorSPX(0);
		
		liftA = new Spark(0);
		liftB = new Spark(1);

		usbCamera = CameraServer.getInstance().startAutomaticCapture(0);
		usbCamera2 = CameraServer.getInstance().startAutomaticCapture(1);

		
		//rearRightMotor.setInverted(true);
		//frontRightMotor.setInverted(true);
		
		// SpeedControllerGroup right = new SpeedControllerGroup(driveCimRF);
		// SpeedControllerGroup left = new SpeedControllerGroup(driveCimLF);
		
		driveTrain = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		//driveTrain.setRightSideInverted(false);
		driveJoystick = new Joystick(0);
		buttonsJoystick = new Joystick(1);
		driveController = new Joystick(2);
		buttonsController = new Joystick(3);
		buttonsJoystickButtons = new HHJoystickButtons(buttonsJoystick, 11);
		driveJoystickButtons = new HHJoystickButtons(driveJoystick, 12);
		
		// ahrs = new AHRS(SerialPort.Port.kMXP.kUSB);
		ahrs = new AHRS(I2C.Port.kOnboard);
		
		ballPusher = new DoubleSolenoid(forwardChannel1, reverseChannel1);
		platePusher = new DoubleSolenoid(forwardChannel2, reverseChannel2);
		//platePusher2 = new DoubleSolenoid(forwardChannel3, reverseChannel3);

		pistonW = new DoubleSolenoid(forwardChannel3, reverseChannel3);
		//pistonW2 = new DoubleSolenoid(forwardChannel4, reverseChannel4);

		climbPistons = new DoubleSolenoid(forwardChannel4, reverseChannel4);

		pdp = new PowerDistributionPanel();

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit(){
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
	}
   /** Need to add teleopInit JF
    * 
    */
	@Override
	public void teleopInit() {
		liftMotor = MotorState.Stopped;
		liftState = LiftState.Bottom;
		wheelIntakeCase = WheelIntakeCase.wheelIntakeStowed;
		isWheelsOut = false;
		frontLeftMotor.stopMotor();
		frontRightMotor.stopMotor();
		rearLeftMotor.stopMotor();
		rearRightMotor.stopMotor();
		driveTrain.driveCartesian(0, 0, 0);
		//ballPusher.set(PUSH);
		platePusher.set(RETRACT);
		enc.reset();
	}
		
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		SmartDashboard.putNumber("Climb Reverse Speed (range -1 to 1)", climbReverseSpeed);
		SmartDashboard.putNumber("Climb Reverse Time (in seconds)", climbReverseTime);
		SmartDashboard.putNumber("Climb Forward Speed", climbForwardSpeed);
		SmartDashboard.putNumber("Climb Forward Time", climbForwardTime);
		SmartDashboard.putNumber("Delay After Climb Pistons Out", delayAfterClimbPistonsOut);
		SmartDashboard.putNumber("Climb Forward Speed After Pistons Out", climbForward2Speed);
		SmartDashboard.putNumber("Climb Forward Time After Pisons Out", climbForward2Time);
		SmartDashboard.putNumber("Delay After Climb Pistons Retract", delayAfterClimbPistonsRetract);

		if (driveController.getRawButtonPressed(8) && !driveControllerSelected)
			driveControllerSelected = true;
		if (driveController.getRawButtonPressed(8) && driveControllerSelected)
			driveControllerSelected = false;
		if (buttonsController.getRawButtonPressed(8) && !buttonsControllerSelected)
			buttonsControllerSelected = true;
		if (buttonsController.getRawButtonPressed(8) && buttonsControllerSelected)
			buttonsControllerSelected = false;
		/*frontLeftMotor.stopMotor();
		frontRightMotor.stopMotor();
		rearLeftMotor.stopMotor();
		rearRightMotor.stopMotor();*/
		if(!driveControllerSelected){//driveControlsSelected.equals(drivejoystick)){
			/*SmartDashboard.putNumber("Drive Joystick X", driveJoystick.getX());
			SmartDashboard.putNumber("Drive Joystick Y", driveJoystick.getY());
			SmartDashboard.putNumber("Drive Joystick Twist", driveJoystick.getTwist());

			if(controller.getRawButton(1)){
				pistonW.set(RETRACT);
			}
			if(controller.getRawButton(2)){
				pistonW.set(PUSH);
			}
			if(buttonsJoystickButtons.isPressed(9)){
				ballPusher.set(RETRACT);
			}
			if(buttonsJoystickButtons.isReleased(9)){
				ballPusher.set(PUSH);
			}
			if(driveJoystick.getY()<(-0.5)){
				frontLeftMotor.set(ControlMode.PercentOutput, -driveJoystick.getTwist());
				rearLeftMotor.set(ControlMode.PercentOutput, -driveJoystick.getTwist());
				frontRightMotor.set(ControlMode.PercentOutput, -driveJoystick.getTwist());
				rearRightMotor.set(ControlMode.PercentOutput, -driveJoystick.getTwist());
			}
			if(!(driveJoystick.getY()==0.5)){
				frontLeftMotor.set(ControlMode.PercentOutput, (-driveJoystick.getTwist()));
				rearLeftMotor.set(ControlMode.PercentOutput, (-driveJoystick.getTwist()));
				frontRightMotor.set(ControlMode.PercentOutput, (-driveJoystick.getTwist()));
				rearRightMotor.set(ControlMode.PercentOutput, (-driveJoystick.getTwist()));
			}
			if(driveJoystick.getTwist()>0.5){
				frontLeftMotor.set(ControlMode.PercentOutput, driveJoystick.getTwist());
				rearLeftMotor.set(ControlMode.PercentOutput, driveJoystick.getTwist());
				frontRightMotor.set(ControlMode.PercentOutput, (-driveJoystick.getTwist()));
				rearRightMotor.set(ControlMode.PercentOutput, (-driveJoystick.getTwist()));
			}
			if(driveJoystick.getTwist()<(-0.5)){
				frontLeftMotor.set(ControlMode.PercentOutput, driveJoystick.getTwist());
				rearLeftMotor.set(ControlMode.PercentOutput, driveJoystick.getTwist());
				frontRightMotor.set(ControlMode.PercentOutput, -driveJoystick.getTwist());
				rearRightMotor.set(ControlMode.PercentOutput, -driveJoystick.getTwist());
			}*/
			if (!(liftState == LiftState.Bottom) && climbState == ClimbState.stopped){
				double xSpeed = (driveJoystick.getX()*0.75);
				double ySpeed = -(driveJoystick.getY()*0.75);
				double zRotation = (driveJoystick.getTwist()*0.75);	

				driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
				driveTrain.setDeadband(driveDeadpan);
			}else if(liftState == LiftState.Bottom && climbState == ClimbState.stopped){
				double xSpeed = driveJoystick.getX();
				double ySpeed = -driveJoystick.getY();
				double zRotation = driveJoystick.getTwist();
				driveTrain.driveCartesian (xSpeed, ySpeed, zRotation);
			}
		
			driveJoystickButtons.updateState();

			BALLINTAKE_BUTTON = driveJoystickButtons.isPressed(1);
			BALLINTAKE_REVERSE_BUTTON_PUSHED = driveJoystickButtons.isPressed(2);
			BALLINTAKE_REVERSE_BUTTON_RELEASED = driveJoystickButtons.isReleased(2);
			CLIMB_BUTTON = driveJoystickButtons.isPressed(12);

		}
		else{// if (driveControlsSelected.equals(drivecontroller)){
			if (!(liftState == LiftState.Bottom)){
				double xSpeed = (driveController.getX() / 5);
				double ySpeed = -(driveController.getY() / 5);
				double zRotation = (driveController.getRawAxis(4) / 5);	

				driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
				driveTrain.setDeadband(driveDeadpan);
			}else{
			double xSpeed = driveController.getX() / 2;
			double ySpeed = -driveController.getY() / 2;
			double zRotation = driveController.getRawAxis(4) / 2;
			
			driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
			driveTrain.setDeadband(driveDeadpan);
			}
			/*BALLINTAKE_BUTTON = driveJoystickButtons.isPressed(1);
			BALLINTAKE_REVERSE_BUTTON_PUSHED = driveJoystickButtons.isPressed(2);
			BALLINTAKE_REVERSE_BUTTON_RELEASED = driveJoystickButtons.isReleased(2);*/
			if(driveController.getRawAxis(3) == 1)
				BALLINTAKE_BUTTON = true;
				//BALLINTAKE_BUTTON = driveController.getRawButtonPressed(1);
			if(driveController.getRawAxis(2) == 1)
				BALLINTAKE_REVERSE_BUTTON_PUSHED = true;
				BALLINTAKE_REVERSE_BUTTON_RELEASED = false;
				//BALLINTAKE_REVERSE_BUTTON_PUSHED = driveController.getRawButtonPressed(2);
			if(driveController.getRawAxis(2) == 0 && BALLINTAKE_REVERSE_BUTTON_PUSHED)
				BALLINTAKE_REVERSE_BUTTON_PUSHED = false;
				BALLINTAKE_REVERSE_BUTTON_RELEASED = true;
			//BALLINTAKE_REVERSE_BUTTON_RELEASED = driveController.getRawButtonReleased(2);
			CLIMB_BUTTON = driveController.getRawButton(7);
		}

		if(!buttonsControllerSelected){//buttonsControlsSelected.equals(buttonsjoystick)){
			SHOOT_BUTTON = buttonsJoystick.getRawButtonPressed(1);
			BALL1_BUTTON = buttonsJoystick.getRawButtonPressed(6);
			BALL2_BUTTON = buttonsJoystick.getRawButtonPressed(7);
			BALL3_BUTTON = buttonsJoystick.getRawButtonPressed(8);
			PLATE1_BUTTON = buttonsJoystick.getRawButtonPressed(11);
			PLATE2_BUTTON = buttonsJoystick.getRawButtonPressed(10);
			PLATE3_BUTTON = buttonsJoystick.getRawButtonPressed(9);
			BALL_CS_BUTTON = buttonsJoystick.getRawButtonPressed(2);
			if(buttonsJoystick.getRawButtonPressed(3) && !liftControl){
				liftControl = true;
				liftState = LiftState.control;
			}
			if(buttonsJoystick.getRawButtonPressed(3) && liftControl){
				liftControl = false;
				liftState = LiftState.Bottom;
			}
		}
		else {//if(buttonsControlsSelected.equals(buttonscontroller)){
			if(buttonsController.getRawButtonPressed(1))
				SHOOT_BUTTON = true;
			if(buttonsController.getRawButtonPressed(5))
				BALL1_BUTTON = true;
			if(buttonsController.getRawAxis(2) == 1)
				BALL2_BUTTON = true;
				//BALL2_BUTTON = buttonsController.getRawButtonPressed(5);
			if(buttonsController.getRawButtonPressed(6))
				BALL3_BUTTON = true;
			if(buttonsController.getRawButtonPressed(2))
				PLATE1_BUTTON = true;
			if(buttonsController.getRawButtonPressed(3))
				PLATE2_BUTTON = true;
			if(buttonsController.getRawButtonPressed(4))
				PLATE3_BUTTON = true;
			if(buttonsController.getRawAxis(3) == 1)
				BALL_CS_BUTTON = true;
				//BALL_CS_BUTTON = buttonsController.getRawButtonPressed(10);
				/*SHOOT_BUTTON = buttonsJoystickButtons.isPressed(1);
				BALL1_BUTTON = buttonsJoystickButtons.isPressed(6);
				BALL2_BUTTON = buttonsJoystickButtons.isPressed(7);
				BALL3_BUTTON = buttonsJoystickButtons.isPressed(8);
				PLATE1_BUTTON = buttonsJoystickButtons.isPressed(11);
				PLATE2_BUTTON = buttonsJoystickButtons.isPressed(10);
				PLATE3_BUTTON = buttonsJoystickButtons.isPressed(9);
				BALL_CS_BUTTON = buttonsJoystickButtons.isPressed(2);*/
			if(buttonsController.getRawButtonPressed(7) && !liftControl){
				liftControl = true;
				liftState = LiftState.control;
			}
			if(buttonsController.getRawButtonPressed(7) && liftControl){
				liftControl = false;
				liftState = LiftState.Bottom;
			}
		}

		SmartDashboard.putString("Lift State", liftState.toString());

		/* joystickA = driveJoystick.getThrottle();
		SmartDashboard.putNumber("Joystick Sensitivity A", joystickA);
		
		joystickY = driveJoystick.getY();
		if (joystickY >= 0) {
			joystickY = joystickB + (1-joystickB)*(joystickA * Math.pow(joystickY, 3) + (1-joystickA)*joystickY);
		}
		else {
			joystickY = -joystickB + (1-joystickB)*(joystickA * Math.pow(joystickY, 3) + (1-joystickA)*joystickY);
		}
	
		joystickZ = driveJoystick.getZ();
		if (joystickZ >= 0) {
			joystickZ = joystickB + (1-joystickB)*(joystickA * Math.pow(joystickZ, 3) + (1-joystickA)*joystickZ);
		}
		else {
			joystickZ = -joystickB + (1-joystickB)*(joystickA * Math.pow(joystickZ, 3) + (1-joystickA)*joystickZ);
		}
		
		
		// driveTrain.arcadeDrive(joystickY,-joystickZ);
		// driveTrain.arcadeDrive(driveJoystick.getY(),driveJoystick.getZ());
		*/
		
		//SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());

		if(BALL1_BUTTON && liftState == LiftState.Bottom){
			height = BALL1_HEIGHT;
			isBall = true;
			liftState = LiftState.Rising;
		}
		if(BALL2_BUTTON && liftState == LiftState.Bottom){
			height = BALL2_HEIGHT;
			isBall = true;
			liftState = LiftState.Rising;
		}
		if(BALL3_BUTTON && liftState == LiftState.Bottom){
			height = BALL3_HEIGHT;
			isBall = true;
			liftState = LiftState.Rising;
		}
		if(PLATE1_BUTTON && liftState == LiftState.Bottom){
			height = PLATE1_HEIGHT;
			isBall = false;
			liftState = LiftState.Rising;
		}
		if(PLATE2_BUTTON && liftState == LiftState.Bottom){
			height = PLATE2_HEIGHT;
			isBall = false;
			liftState = LiftState.Rising;
		}
		if(PLATE3_BUTTON && liftState == LiftState.Bottom){
			height = PLATE3_HEIGHT;
			isBall = false;
			liftState = LiftState.Rising;
		}
		if(BALL_CS_BUTTON && liftState == LiftState.Bottom){
			height = BALL_CS_HEIGHT;
			isBall = true;
			liftState = LiftState.Rising;
		}

		Current = pdp.getCurrent(7);
		double totalCurrent = pdp.getTotalCurrent();
		//SmartDashboard.putNumber("Pdp", Current);
		//SmartDashboard.putNumber("Total Pdp", totalCurrent);

		lift();
		wheelIntake();
		climb();

		//SmartDashboard.putString("Wheel Intake State", wheelIntakeCase.toString());
			
		//SmartDashboard.putNumber("Encoder", enc.getRaw());
		//SmartDashboard.putStr
	}
	
	public void lift(){
		//SmartDashboard.putNumber("Buttons Joystick Y Value", buttonsJoystick.getY());
		//if(buttonsControlsSelected.equals(buttonsjoystick)){
			double a;
			//if(buttonsJoystick.getY()<0){
			//}else{
				//a = (buttonsController.getY() / 4);
			//}
			if(!buttonsControllerSelected)
				a = -buttonsJoystick.getY();
			else
				a = buttonsController.getY();
			//SmartDashboard.putNumber("buttons Joystick y", a);
			//SmartDashboard.putNumber("Height", height);
			//liftA.set(a);
			//liftB.set(a);

			/*if(buttonsJoystick.getRawButtonPressed(1)){
				//ballPusher.set(PUSH);
				platePusher.set(PUSH);
			}
			if(buttonsJoystick.getRawButtonReleased(1))
				platePusher.set(RETRACT);
			if(buttonsJoystick.getRawButtonPressed(2))
				ballPusher.set(PUSH);
			if(buttonsJoystick.getRawButtonReleased(2))
				ballPusher.set(RETRACT);
			/*if(a>0.2){
				liftA.set((a-0.2)*1.25);
				liftB.set((a-0.2)*1.25);
			}
			if(a<-0.2){
				liftA.set((a+0.2)*1.25);
				liftB.set((a-0.2)*1.25);
			}
		}
		else if(buttonsControlsSelected.equals(buttonscontroller)){
			double a = buttonsController.getY();
			double b = buttonsController.getRawAxis(5);
			if(a>0.2){
				liftA.set((a-0.2)*1.25);
				//liftB.set((a-0.2)*1.25);
			}
			if(a<-0.2){
				liftA.set((a+0.2)*1.25);
				//liftB.set((a-0.2)*1.25);
			}
			if(b>0.2 && a<0.2 && a>-0.2){
				liftA.set(((a-0.2)*1.25)/2);
				//liftB.set((a-0.2)*1.25);
			}
			if(b<-0.2 && a<0.2 && a>-0.2){
				liftA.set(((a+0.2)*1.25)/2);
				//liftB.set((a-0.2)*1.25);
			}
		}*/
		/*if(driveJoystick.getRawButton(3)){
			liftA.set(a);
			liftB.set(a);
		}else{*/
		switch (liftState){
			default:
			break;
			case Bottom:
				enc.reset();
				liftMotor = MotorState.Stopped;
				ballPusher.set(RETRACT);
				platePusher.set(RETRACT);
				//platePusher2.set(RETRACT);
				break;
	
			case Rising:
				if(height-(637/3) <= enc.getRaw()){
					liftState = LiftState.FootFromTop;
				}
				else{
				liftMotor = MotorState.Forward;
				//liftA.set(liftSpeed);
				//liftB.set(liftSpeed);
				}
				break;

			case FootFromTop:
				if(height <= getCurrentHeight()){
					liftMotor = MotorState.Stopped;
					liftState = LiftState.Top;
				}else{
					liftMotor = MotorState.Other;
					liftA.set(liftSpeed*0.8);
					liftB.set(liftSpeed*0.8);
				}
				break;
			
			case Top:
				liftMotor = MotorState.Other;
				liftA.set(a);
				liftB.set(a);

				if(SHOOT_BUTTON || PLATE1_BUTTON){
					targetTimeL = Timer.getFPGATimestamp() + delayBeforeRetract;
					liftState = LiftState.Shoot;
				}
				/*if(targetTimeL <= Timer.getFPGATimestamp()){
					targetTimeL = Timer.getFPGATimestamp() + delayBeforeRetract;	
					liftState = LiftState.Shoot;
				}*/
				break;
			
			case Shoot:
				if(isBall){
					ballPusher.set(PUSH);
				}
				else{
					platePusher.set(PUSH);
				}
				/*else if (!isBall && SHOOT_BUTTON){
					ballPusher.set(PUSH);
					//platePusher2.set(PUSH);
				}*/
				if(targetTimeL <= Timer.getFPGATimestamp()){
					liftState = LiftState.Descending;
				}
				break;
	
			case Descending:
				if(getCurrentHeight() <= 637){
					liftState= LiftState.FootFromBottom;
				}
				else{
					liftMotor= MotorState.Reverse;
					ballPusher.set(RETRACT);
					platePusher.set(RETRACT);
					//platePusher2.set(RETRACT);
				}
				break;

			case FootFromBottom:
				if(getCurrentHeight() <= 100){
				liftState = LiftState.Bottom;
				}
				else{
					liftMotor = MotorState.Other;
					liftA.set(-liftSpeed*0.8);
					liftB.set(-liftSpeed*0.8);
				}
				break;
			
			case control:
				liftMotor = MotorState.Other;
				liftA.set(a);
				liftB.set(a);
				if(!buttonsControllerSelected){
					if(buttonsJoystick.getRawButtonPressed(4))
					ballPusher.set(PUSH);
					if(buttonsJoystick.getRawButtonPressed(5))
					platePusher.set(PUSH);
				}
				else{
					if(buttonsController.getRawButtonPressed(9))
					ballPusher.set(PUSH);
					if(buttonsController.getRawButtonPressed(10))
					platePusher.set(PUSH);
				}
				break;
			}

			if(liftMotor == MotorState.Forward){
				liftA.set(liftSpeed);
				liftB.set(liftSpeed);
			}
			else if(liftMotor == MotorState.Reverse){
				liftA.set(-liftSpeed-0.2);
				liftB.set(-liftSpeed-0.2);
			}
			else if (liftMotor == MotorState.Stopped){
				liftA.stopMotor();
				liftB.stopMotor();
			}
		}
			//}

	public double getCurrentHeight(){
		double distance = enc.getRaw();
		return distance;
	}

	public void wheelIntake(){
			
		MotorState wheelintake;

		wheelintake = MotorState.Stopped;
			
		//pistonW.set(PUSH);
		//pistonW2.set(RETRACT);

		//SmartDashboard.putNumber("targetTime", targetTime);
		//SmartDashboard.putNumber("current Time", Timer.getFPGATimestamp());
			
		switch (wheelIntakeCase){
			case wheelIntakeStowed:
				wheelintake = MotorState.Stopped;
				pistonW.set(PUSH);
				//pistonW2.set(PUSH);
				isWheelsOut = false;
				if(BALLINTAKE_BUTTON && !isWheelsOut){
					isWheelsOut = true;
					targetTimeW = Timer.getFPGATimestamp() + delayAfterWheelIntakeOut;
					wheelIntakeCase = WheelIntakeCase.wheelIntakeOffAndOut;
				}
				break;
		
			case wheelIntakeOffAndOut:
				pistonW.set(RETRACT);
				//pistonW2.set(RETRACT);
				if (targetTimeW <= Timer.getFPGATimestamp()){
					wheelIntakeCase = WheelIntakeCase.wheelIntakeOnAndOut;
				}
				break;
			
			case wheelIntakeOnAndOut:
				wheelintake = MotorState.Forward;
				if(BALLINTAKE_REVERSE_BUTTON_PUSHED && isWheelsOut){
					wheelIntakeCase = WheelIntakeCase.wheelIntakeReverse;
				}
				if(BALLINTAKE_BUTTON && isWheelsOut){
					targetTimeW = Timer.getFPGATimestamp() + delayAfterWheelIntakeOut;
					wheelIntakeCase = WheelIntakeCase.wheelIntakeOffAndOut2;
				}	
				break;
			case wheelIntakeOffAndOut2:
				wheelintake = MotorState.Stopped;
				if (targetTimeW <= Timer.getFPGATimestamp()){
					wheelIntakeCase = WheelIntakeCase.wheelIntakeStowed;
				}
				break;
			case wheelIntakeReverse:
				wheelintake = MotorState.Reverse;
				if(BALLINTAKE_REVERSE_BUTTON_RELEASED && isWheelsOut){
					wheelIntakeCase = WheelIntakeCase.wheelIntakeOnAndOut;
				}
				break;
				}
		

		if (wheelintake == MotorState.Forward){
			intake.set(ControlMode.PercentOutput, intakeSpeed);
		}
		if (wheelintake == MotorState.Reverse){
			intake.set(ControlMode.PercentOutput, -intakeSpeed);
		}
		if (wheelintake == MotorState.Stopped){
			intake.set(ControlMode.PercentOutput, 0);
		}
	}

	public void climb(){
		switch (climbState){
			case stopped:
				climbPistons.set(RETRACT);
				if(CLIMB_BUTTON){
					climbState = ClimbState.reverse;
					targetTimeC = Timer.getFPGATimestamp() + SmartDashboard.getNumber("Climb Reverse Time (in Seconds)", climbReverseTime);
				}
				break;
			case reverse:
				driveTrain.driveCartesian(SmartDashboard.getNumber("Climb Reverse Speed (range -1 to 1)", climbReverseSpeed), 0, 0);
				if(targetTimeC <= Timer.getFPGATimestamp()){
					targetTimeC = Timer.getFPGATimestamp() + SmartDashboard.getNumber("Climb Forward Time", climbForwardTime);
					driveTrain.driveCartesian(0, 0, 0);
					climbState = ClimbState.forward;
				}
				break;
			case forward:
				driveTrain.driveCartesian(SmartDashboard.getNumber("Climb Forward Speed", climbForwardSpeed), 0, 0);
				if(targetTimeC <= Timer.getFPGATimestamp()){
					targetTimeC = Timer.getFPGATimestamp() + SmartDashboard.getNumber("Delay After Climb Pistons Out", delayAfterClimbPistonsOut);
					driveTrain.driveCartesian(0, 0, 0);
					climbState = ClimbState.lift;
				}
				break;
			case lift:
				climbPistons.set(PUSH);
				if(targetTimeC <= Timer.getFPGATimestamp()){
					targetTimeC = Timer.getFPGATimestamp() + SmartDashboard.getNumber("Climb Forward Time After Pistons Out", climbForward2Time);
					climbState = ClimbState.forward2;
				}
				break;
			case forward2:
				driveTrain.driveCartesian(SmartDashboard.getNumber("Climb Forward Speed After Pistons Out", climbForward2Speed), 0, 0);
				if(targetTimeC <= Timer.getFPGATimestamp()){
					driveTrain.driveCartesian(0, 0, 0);
					targetTimeC = Timer.getFPGATimestamp() + SmartDashboard.getNumber("Delay After Climb Pistons Retract", delayAfterClimbPistonsRetract);
					climbState = ClimbState.pistonsRetract;
				}
				break;

			case pistonsRetract:
				climbPistons.set(RETRACT);
				if(targetTimeC <= Timer.getFPGATimestamp()){
					targetTimeC = Timer.getFPGATimestamp() + SmartDashboard.getNumber("Climb Forward Time After Pistons Out", climbForward2Time);
					climbState = ClimbState.finalDriveForward;
				}
				break;

			case finalDriveForward:
				driveTrain.driveCartesian(SmartDashboard.getNumber("Climb Forward Speed After Pistons Out", climbForward2Speed), 0, 0);
				if(targetTimeC <= Timer.getFPGATimestamp()){
					driveTrain.driveCartesian(0, 0, 0);
					climbState = ClimbState.stopped;
				}
				break;
		}
	}
			
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	//void usbCamera () {}
	 


	}



