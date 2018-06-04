package connectivity.fri.sdk.example.LBRJointSineOverlay;

import java.util.logging.Logger;

import com.kuka.connectivity.fri.clientSDK.base.ClientApplication;
import com.kuka.connectivity.fri.clientSDK.connection.UdpConnection;

/**
 * Implementation of a FRI client application.
 * <p>
 * The application provides a {@link ClientApplication#connect}, a
 * {@link ClientApplication#step()} and a {@link ClientApplication#disconnect}
 * method, which will be called successively in the application life-cycle.
 * 
 * 
 * @see ClientApplication#connect
 * @see ClientApplication#step()
 * @see ClientApplication#disconnect
 */
public class MyFRIClientApplication
{

    private static final int DEFAULT_PORTID = 30200;
    private static final double DEFAULT_FREQUENCY = 0.25;
    private static final double DEFAULT_AMPLITUDE = 0.04;
    private static final double DEFAULT_FILTER_COEFFICIENT = 0.99;
    private static final int DEFAULT_JOINTMASK = 0x8;

    /**
     * Auto-generated method stub. Do not modify the contents of this method.
     * 
     * @param argv
     *            the arguments
     * 
     * 
     */
    public static void main(String[] argv)
    {
        if (argv.length > 0)
        {
            if (argv[0].equals("help"))
            {
                Logger.getAnonymousLogger().info("\nKUKA LBR joint sine overlay test application\n\n\tCommand line arguments:");
                Logger.getAnonymousLogger().info("\t1) remote hostname (optional)");
                Logger.getAnonymousLogger().info("\t2) port ID (optional)");
                Logger.getAnonymousLogger().info("\t3) bit mask encoding of joints to be overlaid (optional)");
                Logger.getAnonymousLogger().info("\t4) sine frequency in Hertz (optional)");
                Logger.getAnonymousLogger().info("\t5) sine amplitude in radians (optional)");
                Logger.getAnonymousLogger().info("\t6) filter coefficient from 0 (off) to 1 (optional)");
                return;
            }
        }

        String hostname = (argv.length >= 1) ? argv[0] : null;
        int port = (argv.length >= 2) ? Integer.valueOf(argv[1]) : DEFAULT_PORTID;
        int jointMask = (argv.length >= 3) ? Integer.valueOf(argv[2]) : DEFAULT_JOINTMASK;
        double frequency = (argv.length >= 4) ? Double.valueOf(argv[3]) : DEFAULT_FREQUENCY;
        double amplitude = (argv.length >= 5) ? Double.valueOf(argv[4]) : DEFAULT_AMPLITUDE;
        double filterCoeff = (argv.length >= 6) ? Double.valueOf(argv[5]) : DEFAULT_FILTER_COEFFICIENT;

        Logger.getAnonymousLogger().info("Enter LBRJointSineOverlay Client Application");

        /***************************************************************************/
        /*                                                                         */
        /* Place user Client Code here */
        /*                                                                         */
        /**************************************************************************/

        // create new sine overlay client
        LBRJointSineOverlayClient client = new LBRJointSineOverlayClient(jointMask, frequency, amplitude, filterCoeff);

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Configuration */
        /*                                                                         */
        /***************************************************************************/

        // create new udp connection
        UdpConnection connection = new UdpConnection();

        // pass connection and client to a new FRI client application
        ClientApplication app = new ClientApplication(connection, client);

        // connect client application to KUKA Sunrise controller
        app.connect(port, hostname);

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Execution mainloop */
        /*                                                                         */
        /***************************************************************************/

        // repeatedly call the step routine to receive and process FRI packets
        boolean success = true;
        while (success)
        {
            success = app.step();
        }

        /***************************************************************************/
        /*                                                                         */
        /* Standard application structure */
        /* Dispose */
        /*                                                                         */
        /***************************************************************************/

        // disconnect from controller
        app.disconnect();

        Logger.getAnonymousLogger().info("Exit LBRJointSineOverlay Client Application");
    }
}
