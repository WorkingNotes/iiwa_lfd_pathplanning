package lbrExampleApplications;


import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

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
public class MoveTest extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private Tool gripper;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		gripper=getApplicationData().createFromTemplate("Tool_1");
		gripper.attachTo(lbr.getFlange());
	}

	public void run() {
		lbr.move(ptpHome());
		JointTorqueCondition torqueCondJ3=new JointTorqueCondition(JointEnum.J3,-2.5,4.0);
		TorqueSensorData a;
		int k=0;
		
		while(true){
		a=lbr.getExternalTorque();
		getLogger().info(a.toString());
		ThreadUtil.milliSleep(1000);
		k++;
		if(k==10)
		{break;}
		//millsleep}
		}
//		lbr.move(ptp(0.3,0.7,0.4,0.1,0.5,0.6,1));
//		lbr.move(ptpHome());
//		Frame frame_a=new Frame(100,100,100,0,0,0);
//		frame_a.setX(761.29);
//		frame_a.setY(-200.43);
//		frame_a.setZ(493.34);
//		frame_a.setAlphaRad(Math.toRadians(160.61));
//		frame_a.setBetaRad(Math.toRadians(44.56));
//		frame_a.setGammaRad(Math.toRadians(164.29));
//		
//		Frame frame_b=new Frame(100,100,100,0,0,0);
//		frame_b.setX(502.09);
//		frame_b.setY(-115.38);
//		frame_b.setZ(1025.62);
//		frame_b.setAlphaRad(Math.toRadians(38.98));
//		frame_b.setBetaRad(Math.toRadians(41.42));
//		frame_b.setGammaRad(Math.toRadians(30.04));
//		//frame_a.setRedundancyInformation(lbr, 0);
//		frame_a.setParent(getApplicationData().getFrame("/A1"));
//		//frame_a.g
//		lbr.move(ptp(getApplicationData().getFrame("/A1/P1")).setJointVelocityRel(0.3));
//
//		lbr.move(circ(frame_a,frame_b));
//		
//		lbr.move(ptp(frame_a));
//		lbr.move(lin(getApplicationData().getFrame("/A1/P1")));
//		
		
		
		
//		Frame frame_tool_a=new Frame(0,0,0,0,0,0);
//		Frame frame_tool_b=new Frame(0,0,0,0,0,0);
//		frame_tool_a.setX(761.29);
//		frame_tool_a.setY(-200);
//		frame_tool_a.setZ(660.29);
//		frame_tool_a.setAlphaRad(Math.toRadians(160.61));
//		frame_tool_a.setBetaRad(Math.toRadians(44.56));
//		frame_tool_a.setGammaRad(Math.toRadians(164.29));
//		
//		frame_tool_a.setX(761.29);
//		frame_tool_a.setY(-220);
//		frame_tool_a.setZ(660.29);
//		frame_tool_a.setAlphaRad(Math.toRadians(160.61));
//		frame_tool_a.setBetaRad(Math.toRadians(44.56));
//		frame_tool_a.setGammaRad(Math.toRadians(164.29));
//		
//		gripper.getFrame("Frame_Test").move(linRel(0,0,10));
//		gripper.getFrame("Frame_Test").move(lin(frame_tool_b));
	}

	
	private void movePtp(){
	getLogger().info("running PTP");
	double vel;
	}
	
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		MoveTest app = new MoveTest();
		app.runApplication();
	}
}
