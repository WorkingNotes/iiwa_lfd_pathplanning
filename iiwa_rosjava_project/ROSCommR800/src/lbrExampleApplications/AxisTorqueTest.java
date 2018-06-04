package lbrExampleApplications;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;

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
public class AxisTorqueTest extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private ForceCondition normalForce;
	private ForceCondition shearForce;
	private ForceCondition spatialForce;
	private ObjectFrame world = World.Current.getRootFrame();
	private Tool gripper;
	
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		//get world coordinate system
		world = World.Current.getRootFrame();
		
		//initialize the gripper-template
		gripper = getApplicationData().createFromTemplate("GRP_PN_1");
	}

	public void run() {
		gripper.attachTo(lbr.getFlange());
//		lbr.move(ptpHome());
		
//		myTorqueCond();
		lbr.move(ptp(getApplicationData().getFrame("/base1/P1")));
		myForceCondition();
		
		lbr.move(ptp(getApplicationData().getFrame("/base1/P1")));
		
	}
	private void myForceCondition(){
		
//		normalForce = ForceCondition.createNormalForceCondition(gripper.getFrame("TCP_GRP_PN_1"),world, CoordinateAxis.Z, 20);
//		IMotionContainer mc = lbr.move(linRel(0,0,-200,world).breakWhen(normalForce));
//		if (mc.hasFired(normalForce)){
//			getLogger()
//			.info("The normalforce is exceeded! ");
//			getLogger()
//			.info("The normalforce is"+ normalForce.getThreshold()+" N");
//		}
		
//	while(true){
//		shearForce = ForceCondition.createShearForceCondition(gripper.getFrame("TCP_GRP_PN_1"),world, CoordinateAxis.Z, 20);
//		IMotionContainer mc = lbr.move(linRel(200,0,0,world).breakWhen(shearForce));
//		mc = lbr.move(linRel(-200,0,0,world).breakWhen(shearForce));
//		
//		if(mc.hasFired(shearForce)){
//			getLogger().info("The shearforce is exceeded!");
//			getLogger().info("The shearfoce is"+ shearForce.getThreshold()+"N");
//			break;
//		}
//	}
		
		
		while(true){
			spatialForce = ForceCondition.createSpatialForceCondition(gripper.getFrame("TCP_GRP_PN_1"),20);
			IMotionContainer mc =lbr.move(linRel(100,100,100,world).breakWhen(spatialForce));
			mc =lbr.move(linRel(-100,-100,-100,world).breakWhen(spatialForce));
			if(mc.hasFired(spatialForce)){
				getLogger().info("The spatialforce is exceeded!");
				getLogger().info("The spatialfoce is"+ spatialForce.getThreshold()+"N");
				break;
			}
		}
		
		
		}
			

	private void myTorqueCond(){
		lbr.move(ptp(getApplicationData().getFrame("/base1/P1")));
		lbr.move(ptp(getApplicationData().getFrame("/base1/P2")));
		JointTorqueCondition condA1 = new JointTorqueCondition(JointEnum.J1,-2,2);
		JointTorqueCondition condA2 = new JointTorqueCondition(JointEnum.J2,-5,5);
		JointTorqueCondition condA3 = new JointTorqueCondition(JointEnum.J3,-5,5);
		JointTorqueCondition condA4 = new JointTorqueCondition(JointEnum.J4,-5,5);
//		lbr.move(ptp(getApplicationData().getFrame("/base1/P1")));
//		lbr.move(ptp(getApplicationData().getFrame("/base1/P2")));

		ICondition cond = condA1.or(condA2, condA3,condA4);
		IMotionContainer mc = lbr.move(lin(getApplicationData().getFrame("/base1/P3")).breakWhen(condA1));
//        lbr.move(ptp(getApplicationData().getFrame("/base1/P3")));
        
		/*
		try {
			lbr.move(lin(getApplicationData().getFrame("/BlueGear/P3")).breakWhen(cond));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		*/
		if (mc.hasFired(condA1)){
			lbr.move(ptp(getApplicationData().getFrame("/base1/P2")));
			getLogger()
			.info("The Torque of Axis A1 is exceeded! " +
					"going back to point 2");
			
		}

		
		}
		
	
	

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		AxisTorqueTest app = new AxisTorqueTest();
		app.runApplication();
	}
}
