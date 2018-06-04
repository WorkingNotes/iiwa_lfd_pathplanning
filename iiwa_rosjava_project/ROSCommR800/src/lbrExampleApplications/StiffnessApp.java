package lbrExampleApplications;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
/**
 * 
 * @author schick
 * App testing impedanceController by using userkeys 
 */
public class StiffnessApp extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
//	private Gripper gripperIO;
	private Tool gripper;
	private double stiffness = 1000;
	private CartesianImpedanceControlMode mode;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getRobot(kuka_Sunrise_Cabinet_1, "LBR_iiwa_7_R800_1");

		gripper = getApplicationData().createFromTemplate("Gripper");
//		gripperIO = new Gripper(kuka_Sunrise_Cabinet_1);
//		gripperIO.setSensorOn(false);

		mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.TRANSL).setStiffness(stiffness);
		mode.parametrize(CartDOF.ROT).setStiffness(100);
		mode.parametrize(CartDOF.ALL).setDamping(0.7);

	}

	public void run() {
		getLogger().info("Start " + this.getClass().getName());
		setStiffnessBar();

		gripper.attachTo(lbr.getFlange());
		// gripper.move(ptpHome());

		gripper.move(ptp(getApplicationData().getFrame("/VorPos")));

		// gripper.move(positionHold(mode, -1, TimeUnit.SECONDS));
		while (true) {
			gripper.move(lin(getApplicationData().getFrame("/VorPos"))
					.setCartVelocity(0.1).setMode(mode));
			mode.parametrize(CartDOF.TRANSL).setStiffness(getStiffness());
			mode.parametrize(CartDOF.ROT).setStiffness(100);
			mode.parametrize(CartDOF.ALL).setDamping(0.1);
			ThreadUtil.milliSleep(100);
		}

		// getLogger().info("End " + this.getClass().getName());
	}

	private void setStiffnessBar() {
		IUserKeyBar stiffBar = getApplicationUI().createUserKeyBar("Stiff");

		IUserKeyListener stiffnessListener10 = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {

				switch (event) {
				case FirstKeyDown:

					stiffness += 10;
					key.setText(UserKeyAlignment.Middle,
							String.valueOf(getStiffness()));
					IUserKey key100 = getApplicationUI().getUserKeyBar("Stiff")
							.getUserKey(2);
					key100.setText(UserKeyAlignment.Middle,
							String.valueOf(stiffness));
					mode.parametrize(CartDOF.TRANSL).setStiffness(
							getStiffness());

					break;
				case SecondKeyDown:

					stiffness -= 10;
					key.setText(UserKeyAlignment.Middle,
							String.valueOf(getStiffness()));
					IUserKey key101 = getApplicationUI().getUserKeyBar("Stiff")
							.getUserKey(2);
					key101.setText(UserKeyAlignment.Middle,
							String.valueOf(stiffness));
					mode.parametrize(CartDOF.TRANSL).setStiffness(
							getStiffness());

					break;
				}
			}
		};
		IUserKeyListener stiffnessListener100 = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {

				switch (event) {
				case FirstKeyDown:

					stiffness += 100;
					key.setText(UserKeyAlignment.Middle,
							String.valueOf(getStiffness()));
					IUserKey key10 = getApplicationUI().getUserKeyBar("Stiff")
							.getUserKey(0);
					key10.setText(UserKeyAlignment.Middle,
							String.valueOf(stiffness));
					mode.parametrize(CartDOF.TRANSL).setStiffness(
							getStiffness());

					break;
				case SecondKeyDown:

					stiffness -= 100;
					key.setText(UserKeyAlignment.Middle,
							String.valueOf(getStiffness()));
					IUserKey key11 = getApplicationUI().getUserKeyBar("Stiff")
							.getUserKey(0);
					key11.setText(UserKeyAlignment.Middle,
							String.valueOf(stiffness));
					mode.parametrize(CartDOF.TRANSL).setStiffness(
							getStiffness());

					break;
				}
			}
		};

		IUserKey stiffKey10 = stiffBar.addDoubleUserKey(0, stiffnessListener10,
				false);
		IUserKey stiffKey100 = stiffBar.addDoubleUserKey(2,
				stiffnessListener100, false);

		stiffKey10.setText(UserKeyAlignment.TopMiddle, "+10");
		stiffKey10.setText(UserKeyAlignment.BottomMiddle, "-10");
		stiffKey10.setText(UserKeyAlignment.Middle, String.valueOf(stiffness));

		stiffKey100.setText(UserKeyAlignment.TopMiddle, "+100");
		stiffKey100.setText(UserKeyAlignment.BottomMiddle, "-100");
		stiffKey100.setText(UserKeyAlignment.Middle, String.valueOf(stiffness));

		stiffBar.publish();
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		StiffnessApp app = new StiffnessApp();
		app.runApplication();
	}

	/**
	 * @return stiffness
	 */
	public double getStiffness() {
		return stiffness;
	}

	/**
	 * @param stiffness
	 *            das zu setzende Objekt stiffness
	 */
	public void setStiffness(double stiffness) {
		if (stiffness > 5000)
			this.stiffness = 5000;
		if (stiffness < 0)
			this.stiffness = 0;
		this.stiffness = stiffness;
	}
}
