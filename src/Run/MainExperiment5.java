package Run;

import Scheme.DeployedServices;
import Scheme.Parameters;
import DTMC.DTMCconstructor;
import DTMC.DTMCsimulator;
import Scheme.ServiceDeployMethod;
import Components.Method;
import Components.Traffic;
import Components.Violation;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.Arrays;

/**
 *
 * @author Ashkan Y.
 *
 * This is the main class for experiment 5
 */
public class MainExperiment5 {

    private final static int TOTAL_RUN = 2;
    private final static int TAU = 10; // time interval between run of the method (s)
    private final static int TRAFFIC_CHANGE_INTERVAL = 5; // time interval between run of the method (s)

    public static void main(String[] args) throws IOException {
        // in each experiment, these parameters may vary
        Parameters.MEASURING_RUNNING_TIME = true;

        Parameters.numCloudServers = 3;
        Parameters.numFogNodes = 100;
        Parameters.numServices = 10000;
        Traffic.TRAFFIC_ENLARGE_FACTOR = 1;

        DTMCconstructor dtmcConstructor = new DTMCconstructor();
        DTMCsimulator trafficRateSetter = new DTMCsimulator(dtmcConstructor.dtmc);

        FileWriter fw = new FileWriter("/home/soroush/Desktop/FogPlan/expr5_results.txt");

        int[] fogNodes = {100, 100, 100, 100, 1000, 10000};
        int[] numServices = {100, 1000, 10000, 100, 100, 100};
        long[] runTime = new long[6];

        System.out.println("Time\t#Fog\t#Service\titer\tDelay");
        double[][] arrayOfDelays = new double[10][6];
        for(int conf = 0; conf < 6; conf++) {
            for (int time = 0; time < 10; time++) {
                Parameters.numFogNodes = fogNodes[conf];
                Parameters.numServices = numServices[conf];
                Parameters.initialize();
                Parameters.TAU = TAU;
                Parameters.TRAFFIC_CHANGE_INTERVAL = TRAFFIC_CHANGE_INTERVAL;

                Method PSObased = new Method(new ServiceDeployMethod(ServiceDeployMethod.PSOCRO), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
                double[] delayPSOCRO = new double[TOTAL_RUN];
                for (int i = 0; i < TOTAL_RUN; i++) {
                    double trafficPerNodePerApp = trafficRateSetter.nextRate();
                    Traffic.distributeTraffic(trafficPerNodePerApp);

                    Traffic.setTrafficToGlobalTraffic(PSObased);
                    Instant start = Instant.now();
                    PSObased.run(Traffic.AGGREGATED, false);
                    runTime[conf] += Duration.between(start, Instant.now()).getSeconds();
                    delayPSOCRO[i] = PSObased.getAvgServiceDelay();

                    System.out.println(time + "\t" + Parameters.numFogNodes + "\t" + Parameters.numServices + "\t" + i + "\t" + delayPSOCRO[i]);
                }

                arrayOfDelays[time][conf] = MainExperiment1new.arrayAverage(delayPSOCRO);
            }

            runTime[conf] /= TOTAL_RUN * 10;
        }

        double[] avgDelays = new double[6];
        for(int i = 0; i<10; i++)
            for(int j = 0; j<6; j++)
                avgDelays[i*6 + j] = arrayOfDelays[i][j];

//        fw.write(Arrays.toString(runTime) + " " + );

        fw.close();
    }

}
