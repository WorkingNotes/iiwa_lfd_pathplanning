package lbrExampleApplications;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * @author micheler
 *
 */
public class FeatureDemo extends RoboticsAPIApplication {
	private Controller cab_1;
	private LBR iiwa7_1;
	private Tool _tool;

	public void initialize() {
		cab_1 = getController("KUKA_Sunrise_Cabinet_1");
		iiwa7_1 = (LBR) getRobot(cab_1, "LBR_iiwa_7_R800_1");
		_tool = getApplicationData().createFromTemplate("Gripper");
		_tool.attachTo(iiwa7_1.getFlange());

	}

	public void run() {
				
		int sel = 10;
		while (sel != 8) {
			sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
				"Select Application? (ATTENTION: Robot will move!)",
				"Startposition", // 0
				"cartesian movement", // 1
				"kinematic redundancy", // 2
				"impedance control mode", // 3
				"wiggle & bounce", // 4
				"collision detection", // 5
				"demo in loop", // 6
				"Teaching by demonstration", // 7
				"Exit application");//8
			
			switch(sel) {
				case 0:
					_tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
					break;
				case 1:
					moveCartesian(2);
					break;
				case 2:
					moveNullspace(2);
					break;
				case 3:
					stiffness();
					break;
				case 4:
					wiggleBounce();
					break;
				case 5:
					collisionDetection(2);
					break;
				case 6:
					demonstration();
					break;
				case 7:
					teaching();
					break;
				default:
					_tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
					break;			
			}
		}
	}

	private void moveCartesian(int runs) {
		getLogger().info("moveCartesian");

		//collision condition definition
		ICondition forceCon = defineSensitivity();

		//motion programming
		_tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		MotionBatch cart = new MotionBatch(
			lin(getFrame("/start")).setCartVelocity(100),
			linRel(-150, 0, 0).setCartVelocity(100),//x direction 
			lin(getFrame("/start")).setCartVelocity(100),
			linRel(0, 150, 0).setCartVelocity(100),//y direction
			lin(getFrame("/start")).setCartVelocity(100),
			linRel(0, 0, 150).setCartVelocity(100),//z direction
			lin(getFrame("/start")).setCartVelocity(100))
		.breakWhen(forceCon);
		
		IMotionContainer motion;
		for (int i = 0; i < runs; i++) {
			motion = _tool.move(cart);
			if (motion.hasFired(forceCon)) {
				//Reaction after collision
				boolean resumeMotion = behaviourAfterCollision();
				if (!resumeMotion) break;
			}
		}
	}
	
	private void moveNullspace(int runs) {
		getLogger().info("Nullspace Movement");
		
		//collision condition definition
		ICondition forceCon = defineSensitivity();

		//motion programming
		_tool.move(ptp(getFrame("/start")).setJointVelocityRel(.5));
		MotionBatch ns = new MotionBatch(
			lin(getFrame("/start/Links")),
			lin(getFrame("/start/Rechts")),
			lin(getFrame("/start/Links")),
			lin(getFrame("/start")))
		.setJointVelocityRel(0.3).breakWhen(forceCon);
		
		IMotionContainer motion;
		for (int i = 0; i < runs; i++) {
			motion = _tool.move(ns);
			if (motion.hasFired(forceCon)) {
				//Reaction after collisionn
				boolean resumeMotion = behaviourAfterCollision();
				if (!resumeMotion) break;
			}
		}
	}
	
	private void stiffness() {
		getLogger().info("Stiffness");
		
		int answer = 0;
		double stiffP = 1000.0;
		double stiffR = 200.0;

		double highStiff = getApplicationData().getProcessData("highStiff").getValue();
		double midStiff = getApplicationData().getProcessData("midStiff").getValue();
		double lowStiff = getApplicationData().getProcessData("lowStiff").getValue();

		
		_tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		CartesianImpedanceControlMode modeHandfuehren = new CartesianImpedanceControlMode();
		do {
			switch (answer) {
			case 0:
				stiffP = 1000.0;
				stiffR = 200.0;
				break;
			case 1:
				stiffP = lowStiff;
				stiffR = lowStiff/10;
				break;
			case 2:
				stiffP = midStiff;
				stiffR = midStiff/10;
				break;
			case 3:
				stiffP = highStiff;
				stiffR = highStiff/10;
				break;
			}
			modeHandfuehren.parametrize(CartDOF.TRANSL).setStiffness(stiffP);
			modeHandfuehren.parametrize(CartDOF.ROT).setStiffness(stiffR);

			IMotionContainer handle;
				handle = _tool.moveAsync(positionHold(modeHandfuehren, -1, TimeUnit.SECONDS));

				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.INFORMATION,
						"Stiffness in Translation: " + stiffP + " N/m" +
						"Stiffness in Rotation:    " + stiffR + " N/rad",
						"Move to Startposition and End", "LowStiffness", "MidStiffness", "HighStiffness");
				handle.cancel();
				_tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		} while (answer != 0);
	}	
	
	private void wiggleBounce() {
		getLogger().info("Wiggle & Bounce");
		
		int sel = 0;
		String actSwing = "";
		
		//define condition
		ICondition forceCon = defineSensitivity();

		//sine wave
		// Sine Frequency 2 Hz, Amplitude 50 N, Stiffness 1500 [N/m]
		CartesianSineImpedanceControlMode shakeSinX;
		shakeSinX = CartesianSineImpedanceControlMode.createSinePattern(CartDOF.X, 2, 50, 1500);
		shakeSinX.parametrize(CartDOF.ALL).setDamping(0.7);
		
		// Sine Frequency 5 Hz, Amplitude 5 Nm, Stiffness 15 [Nm/rad]
		CartesianSineImpedanceControlMode shakeSinA;
		shakeSinA = CartesianSineImpedanceControlMode.createSinePattern(CartDOF.A, 3, 5, 15);
		shakeSinA.parametrize(CartDOF.ALL).setDamping(0.7);
		
		// Lissajous-wave Frequency 1 Hz, Amplitude 50 N, Stiffness 1500 [N/m]
		CartesianSineImpedanceControlMode shakeLis;
		shakeLis = CartesianSineImpedanceControlMode.createLissajousPattern(CartPlane.XY, 1, 50, 1500);
		shakeLis.parametrize(CartDOF.Z).setStiffness(1000); 		// ... mit Steifigkeit 1000 [N/m] in Z-Richtung
		shakeLis.setRiseTime(3);									// ... ueber 3 Sekunden ansteigend
		
		// Spiral-Shack Frequency 15 Hz, Amplitude 16 N, Stiffness 1000 [N/m]
		CartesianSineImpedanceControlMode shakeSpirale;
		shakeSpirale = CartesianSineImpedanceControlMode.createSpiralPattern(CartPlane.XY, 15, 16, 1000, 180);
		shakeSpirale.setRiseTime(0.2).setHoldTime(60).setFallTime(0.5);
		
		
		//motion programming
		IMotionContainer handle;
		handle = _tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		
		while (sel != 4) {
			switch (sel){
				case 0:
					actSwing = "Sine wave in X-direction with 2 Hz";
					handle = _tool.moveAsync(positionHold(shakeSinX, -1, TimeUnit.SECONDS).breakWhen(forceCon));
					break;
				case 1:
					actSwing = "Wave around Tool Z with 3 Hz";
					handle = _tool.moveAsync(positionHold(shakeSinA, -1, TimeUnit.SECONDS).breakWhen(forceCon));
					break;
				case 2:
					actSwing = "Lissajousfigure in XY-surface";
					handle = _tool.moveAsync(positionHold(shakeLis, -1, TimeUnit.SECONDS).breakWhen(forceCon));
					break;
				case 3:
					actSwing = "Spirale in XY-surface";
					handle = _tool.moveAsync(positionHold(shakeSpirale, -1, TimeUnit.SECONDS).breakWhen(forceCon));
					break;
				default:
					break;				
			}
			sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
					"Choose next Shake\n\nNow:" + actSwing, "Sine X 2 Hz", "Sine A 3 Hz", "Lissajous XY 1 Hz", "Spatial XY 15 Hz", "END");	
			handle.cancel();
			_tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		}

		_tool.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
	}
	
	private void collisionDetection(int runs) {
		getLogger().info("Collision Detection");
		
		//collision condition definition
		ICondition forceCon = defineSensitivity();
		
		//motion programming
		Spline liegendeAcht = new Spline(
			spl(getFrame("/start/P2")), spl(getFrame("/start/P3")),
			spl(getFrame("/start/P1")), spl(getFrame("/start/P4")),
			spl(getFrame("/start/P5")), spl(getFrame("/start/P1")),
			spl(getFrame("/start/P2")), spl(getFrame("/start/P3")),
			spl(getFrame("/start/P1")), spl(getFrame("/start/P4")),
			spl(getFrame("/start/P5")), spl(getFrame("/start/P1"))
		).setCartVelocity(150).breakWhen(forceCon);
		
		IMotionContainer motion;
		while(true){
			motion = _tool.move(liegendeAcht);
			if (motion.hasFired(forceCon)) {
				//Reaction after collision
				boolean resumeMotion = behaviourAfterCollision();
				if (!resumeMotion) break;
			}
		}
//		for (int i = 0; i < runs; i++) {
//			motion = _tool.move(liegendeAcht);
//			if (motion.hasFired(forceCon)) {
//				//Reaction after collision
//				boolean resumeMotion = behaviourAfterCollision();
//				if (!resumeMotion) break;
//			}
//		}
	}

	private void demonstration() {
		do {
			int sel = (int) Math.abs(Math.random()*3);
			switch (sel) {
				case 0:
					moveCartesian(1);
					break;
				case 1:
					moveNullspace(1);
					break;
				case 2:
					collisionDetection(1);
					break;
				default:
					break;
			}
		} while (true);	
	}
	
	private void teaching(){
		CartesianImpedanceControlMode mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.ALL).setStiffness(80);
		mode.parametrize(CartDOF.ROT).setStiffness(10);
		
		double blending = getApplicationData().getProcessData("blending").getValue();
		int tick = getApplicationData().getProcessData("tick").getValue();
		ArrayList<JointPosition> positions = new ArrayList<JointPosition>(); // save teaching position info.
		
		JointTorqueCondition fc = new JointTorqueCondition(JointEnum.J1, -15, 15);
		
		iiwa7_1.move(ptp(0, Math.toRadians(25), 0, Math.toRadians(-85), 0, Math.toRadians(70), 0).setJointVelocityRel(.3).setJointAccelerationRel(.5));
		
		iiwa7_1.move(positionHold(new PositionControlMode(), -1, TimeUnit.SECONDS).breakWhen(fc));
		
		int time = getApplicationData().getProcessData("time").getValue();
		
		IMotionContainer posHold = iiwa7_1.moveAsync(positionHold(mode, time, TimeUnit.SECONDS));//time of recording
		
		ThreadUtil.milliSleep(1000);
		while(!posHold.isFinished()){
			ThreadUtil.milliSleep(tick);//frequency of recording
			positions.add(iiwa7_1.getCurrentJointPosition());
		}
		
		//ThreadUtil.milliSleep(2000);
		iiwa7_1.move(ptp(iiwa7_1.getCurrentJointPosition()));
		
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Play recorded path", "Play");
		int sel = 10;
		while (sel !=1){
			iiwa7_1.move(ptp(positions.get(0)).setJointVelocityRel(.2));
			
			double velo = getApplicationData().getProcessData("velo").getValue();
			double acce = getApplicationData().getProcessData("acce").getValue();
			double jerk = getApplicationData().getProcessData("jerk").getValue();
			
			for(int i = 1; i < positions.size(); i++){
				iiwa7_1.moveAsync(ptp(positions.get(i)).setBlendingRel(blending).setJointVelocityRel(velo).setJointJerkRel(jerk).setJointAccelerationRel(acce));
			}
			sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Replay or End?", "Replay", "End");
		}
				
		iiwa7_1.move(ptp(positions.get(0)).setJointVelocityRel(.1));
	}
	
	private ICondition defineSensitivity() {
		double sensCLS = getApplicationData().getProcessData("sensCLS").getValue();
		getLogger().info("Sensitive in each axis: " +sensCLS + " Nm\nShow current torque processdata in each axis.");
		
		//Offset compensation
		double actTJ1 = iiwa7_1.getExternalTorque().getSingleTorqueValue(JointEnum.J1);
		double actTJ2 = iiwa7_1.getExternalTorque().getSingleTorqueValue(JointEnum.J2);
		double actTJ3 = iiwa7_1.getExternalTorque().getSingleTorqueValue(JointEnum.J3);
		double actTJ4 = iiwa7_1.getExternalTorque().getSingleTorqueValue(JointEnum.J4);
		double actTJ5 = iiwa7_1.getExternalTorque().getSingleTorqueValue(JointEnum.J5);
		double actTJ6 = iiwa7_1.getExternalTorque().getSingleTorqueValue(JointEnum.J6);
		double actTJ7 = iiwa7_1.getExternalTorque().getSingleTorqueValue(JointEnum.J7);
		
		getLogger().info("OffsetValue\nJ1 " + actTJ1 + "Nm\nJ2 " + actTJ2 + "Nm\nJ3 " + actTJ3 + "Nm\nJ4 " + actTJ4 + "Nm\nJ5 " + actTJ5 + "Nm\nJ6 " + actTJ6 + "Nm\nJ7 " + actTJ7 + "Nm");
		
		//condition on each axis
		JointTorqueCondition jt1 = new JointTorqueCondition(JointEnum.J1, -sensCLS+actTJ1, sensCLS+actTJ1);
		JointTorqueCondition jt2 = new JointTorqueCondition(JointEnum.J2, -sensCLS+actTJ2, sensCLS+actTJ2);
		JointTorqueCondition jt3 = new JointTorqueCondition(JointEnum.J3, -sensCLS+actTJ3, sensCLS+actTJ3);
		JointTorqueCondition jt4 = new JointTorqueCondition(JointEnum.J4, -sensCLS+actTJ4, sensCLS+actTJ4);
		JointTorqueCondition jt5 = new JointTorqueCondition(JointEnum.J5, -sensCLS+actTJ5, sensCLS+actTJ5);
		JointTorqueCondition jt6 = new JointTorqueCondition(JointEnum.J6, -sensCLS+actTJ6, sensCLS+actTJ6);
		JointTorqueCondition jt7 = new JointTorqueCondition(JointEnum.J7, -sensCLS+actTJ7, sensCLS+actTJ7);

		ICondition forceCon = jt1.or(jt2, jt3, jt4, jt5, jt6, jt7);
		return forceCon;
	}

	private boolean behaviourAfterCollision() {
		boolean resumeMotion = true; 
		int sel = 0;
		IMotionContainer handle;
		
		CartesianImpedanceControlMode soft = new CartesianImpedanceControlMode();
			soft.parametrize(CartDOF.ALL).setDamping(.7);
			soft.parametrize(CartDOF.ROT).setStiffness(100);
			soft.parametrize(CartDOF.TRANSL).setStiffness(600);
				
		handle = _tool.moveAsync(positionHold(soft, -1, TimeUnit.SECONDS));
		sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
			"Collision Dection and LBR act gentle",
			"Move Continue",//0
			"return");//1
		handle.cancel();
		if (sel != 0) {
			resumeMotion = false;
			_tool.move(ptp(getFrame("/start")).setJointVelocityRel(.3));
		}
				
		return resumeMotion;
	}	
	
	
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		FeatureDemo app = new FeatureDemo();
		app.runApplication();
	}

}
