package lbrExampleApplications;
import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

import static com.kuka.roboticsAPI.motionModel.MMCMotions.*;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application life cycle. The application will terminate automatically after 
 * the {@link 
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
public class HandGuidance extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR robot;
	private HandGuidingMotion motion;
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		robot = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		// decalare paramerters for the handguiding motion, to avoid Singularities and axes limits
	      motion = new HandGuidingMotion();
	      motion.setPermanentPullOnViolationAtStart(true);
	}
	
	public void run() 
	{
        robot.move(ptp(0, Math.toRadians(20), 0, Math.toRadians(-90), 0,
				Math.toRadians(-20), 0).setJointVelocityRel(0.2));

        while(true)
        {
        	robot.move(motion);
        }

	}
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		HandGuidance app = new HandGuidance();
		app.runApplication();
	}
}