package lbrExampleApplications;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class ImpendenceControl extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	
	final static double offsetAxis2And4=Math.toRadians(10);
	
	private   double stiffnessZ = getApplicationData().getProcessData("stiffnessZ").getValue();
	private   double stiffnessY = getApplicationData().getProcessData("stiffnessY").getValue();
	private   double stiffnessX = getApplicationData().getProcessData("stiffnessX").getValue();
	private   double stiffnessA = getApplicationData().getProcessData("stiffnessA").getValue();
	private   double stiffnessB = getApplicationData().getProcessData("stiffnessB").getValue();
	private   double stiffnessC = getApplicationData().getProcessData("stiffnessC").getValue();
	
	private static double[] startPosition=new double[]{0,offsetAxis2And4,0,offsetAxis2And4-Math.toRadians(90),0,Math.toRadians(90),0};
	

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
	}

	public void run() {
		lbr.move(ptpHome().setJointVelocityRel(0.25));
		impControl();
		wiggleBounce();
		
	}
	
	private void impControl(){
//		CartesianImpedanceControlMode mode = new CartesianImpedanceControlMode();
//		mode.parametrize(CartDOF.X).setStiffness(stiffnessX);
//		mode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
//		mode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
//		mode.parametrize(CartDOF.A).setStiffness(stiffnessA);
//		mode.parametrize(CartDOF.B).setStiffness(stiffnessB);
//		mode.parametrize(CartDOF.C).setStiffness(stiffnessC);
//		lbr.move(ptp(getApplicationData().getFrame("/base1/P1")));
////		lbr.move(positionHold(mode, -20, TimeUnit.SECONDS));
//		
//		// The robot is set to position hold and impedance control mode gets activated without a timeout. 
//		IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(mode, -1, null)));
//		
//		lbr.move(ptp(getApplicationData().getFrame("/base1/P2")));
//		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
//		positionHoldContainer.cancel();
		
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "start ?", "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to start position");
		PTP ptpToStartPosition = ptp(startPosition);
		ptpToStartPosition.setJointVelocityRel(0.25);
		lbr.move(ptpToStartPosition);

		getLogger().info("Hold position in impedance control mode");
		CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		impedanceControlMode.parametrize(CartDOF.A).setStiffness(stiffnessA);
		impedanceControlMode.parametrize(CartDOF.B).setStiffness(stiffnessB);
		impedanceControlMode.parametrize(CartDOF.C).setStiffness(stiffnessC);

		// The robot is set to position hold and impedance control mode gets activated without a timeout. 
		IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(impedanceControlMode, -1, null)));

		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.", "OK");

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();
		
	}

	
	private void wiggleBounce() {
		getLogger().info("Wiggle & Bounce");
		
		int sel = 0;
		String actSwing = "";
		
		//define condition
		ICondition forceCon = defineSensitivity();

		//sine wave
		// Sine Frequency 2 Hz, Amplitude 50 N, Stiffness 1500 [N/m]
		CartesianSineImpedanceControlMode shakeSinY;
		shakeSinY = CartesianSineImpedanceControlMode.createSinePattern(CartDOF.Y, 2, 50, 1500);
		shakeSinY.parametrize(CartDOF.ALL).setDamping(0.7);
		
		// Lissajous-wave Frequency 1 Hz, Amplitude 50 N, Stiffness 1500 [N/m]
		CartesianSineImpedanceControlMode shakeLis;
		shakeLis = CartesianSineImpedanceControlMode.createLissajousPattern(CartPlane.XY, 1, 50, 1500);
		shakeLis.parametrize(CartDOF.Z).setStiffness(1000); 		// ... mit Steifigkeit 1000 [N/m] in Z-Richtung
		shakeLis.setRiseTime(0.2).setHoldTime(60).setFallTime(0.5); 
		
		// Spiral-Shack Frequency 15 Hz, Amplitude 16 N, Stiffness 1000 [N/m]
		CartesianSineImpedanceControlMode shakeSpirale;
		shakeSpirale = CartesianSineImpedanceControlMode.createSpiralPattern(CartPlane.XY, 15, 16, 1000, 180);
		shakeSpirale.parametrize(CartDOF.Z).setStiffness(1000);
		shakeSpirale.setRiseTime(0.2).setHoldTime(60).setFallTime(0.5);
		
		
		sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
				"Choose next Shake\n\nNow:" + actSwing, "Sine Y 2 Hz","Lissajous-wave 1 Hz","Spiral-Shack 15 Hz","END");	
		
		//motion programming
		IMotionContainer handle;
		handle = lbr.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		
		while (sel != 3) {
			switch (sel){
				case 0:
					actSwing = "Sine wave in Y-direction with 2 Hz";
					handle = lbr.moveAsync(positionHold(shakeSinY, -1, TimeUnit.SECONDS).breakWhen(forceCon));
					break;
				case 1:
					actSwing = "Lissajous-wave 1 Hz";
					handle = lbr.moveAsync(positionHold(shakeLis, -1, TimeUnit.SECONDS).breakWhen(forceCon));
					break;
				case 2: 
					actSwing = "Spiral-Shack 15 Hz";
					handle = lbr.moveAsync(positionHold(shakeSpirale, -1, TimeUnit.SECONDS).breakWhen(forceCon));
					break;
				default:
					break;				
			}
			
			sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
					"Choose next Shake\n\nNow:" + actSwing, "Sine Y 2 Hz","Lissajous-wave 1 Hz","Spiral-Shack 15 Hz","END");
			
			handle.cancel();
			lbr.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		}

		lbr.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		
	}
	
	
	private ICondition defineSensitivity() {
		double sensCLS = getApplicationData().getProcessData("sensCLS").getValue();
		getLogger().info("Sensitive in each axis: " +sensCLS + " Nm\nShow current torque processdata in each axis.");
		
		//Offset compensation
		double actTJ1 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J1);
		double actTJ2 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J2);
		double actTJ3 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J3);
		double actTJ4 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J4);
		double actTJ5 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J5);
		double actTJ6 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J6);
		double actTJ7 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J7);
		
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

	

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		ImpendenceControl app = new ImpendenceControl();
		app.runApplication();
	}
}
