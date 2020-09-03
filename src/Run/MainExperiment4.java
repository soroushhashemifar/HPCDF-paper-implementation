package Run;

import Scheme.Parameters;
import DTMC.DTMCconstructor;
import DTMC.DTMCsimulator;
import Scheme.DeployedServices;
import Scheme.ServiceDeployMethod;
import Components.Delay;
import Components.Method;
import Components.Traffic;
import Components.Violation;
import Utilities.Statistics;

import java.io.FileWriter;
import java.io.IOException;

/**
 *
 * @author Ashkan Y. 
 * 
 * This is the main class for experiment 4
 */
public class MainExperiment4 {

    private static int MAX_CHANGE_INTERVAL = 200;
    private final static int TRAFFIC_CHANGE_INTERVAL = 10; // time interval between run of the method (s)
    private static int MIN_CHANGE_INTERVAL = 10;

    private final static int TOTAL_RUN = 200;

    public static void main(String[] args) throws IOException {
        // in each experiment, these parameters may vary
        Parameters.numCloudServers = 3;
        Parameters.numFogNodes = 15;
        Parameters.numServices = 50;

        Parameters.initialize();
        
        Traffic.TRAFFIC_ENLARGE_FACTOR = 4;

        DTMCconstructor dtmcConstructor = new DTMCconstructor();
        DTMCsimulator trafficRateSetter = new DTMCsimulator(dtmcConstructor.dtmc);

        int q; // the number of times that traffic changes between each run of the method
        Parameters.TRAFFIC_CHANGE_INTERVAL = TRAFFIC_CHANGE_INTERVAL;

        Method MinCost = new Method(new ServiceDeployMethod(ServiceDeployMethod.FOG_DYNAMIC), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method MinViol = new Method(new ServiceDeployMethod(ServiceDeployMethod.FOG_DYNAMIC), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method PSObased = new Method(new ServiceDeployMethod(ServiceDeployMethod.PSOCRO), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);

        DeployedServices MCDeployedServices = null;
        DeployedServices MVDeployedServices = null;
        DeployedServices containersDeployedPSOCRO = null;

        Parameters.MEASURING_RUNNING_TIME = false;
        
        double[] fogContainersDeployedMinCost = new double[TOTAL_RUN]; // used for getting average
        double[] cloudContainersDeployedMinCost = new double[TOTAL_RUN]; // used for getting average
        double[] delayMinCost = new double[TOTAL_RUN]; // used for getting average
        double[] costMinCost = new double[TOTAL_RUN]; // used for getting average
        double[] violMinCost = new double[TOTAL_RUN]; // used for getting average

        double[] fogContainersDeployedMinViol = new double[TOTAL_RUN]; // used for getting average
        double[] cloudContainersDeployedMinViol = new double[TOTAL_RUN]; // used for getting average
        double[] delayMinViol = new double[TOTAL_RUN]; // used for getting average
        double[] costMinViol = new double[TOTAL_RUN]; // used for getting average
        double[] violMinViol = new double[TOTAL_RUN]; // used for getting average

        double[] fogContainersDeployedPSOCRO = new double[TOTAL_RUN]; // used for getting average
        double[] cloudContainersDeployedPSOCRO = new double[TOTAL_RUN]; // used for getting average
        double[] delayPSOCRO = new double[TOTAL_RUN]; // used for getting average
        double[] costPSOCRO = new double[TOTAL_RUN]; // used for getting average
        double[] violPSOCRO = new double[TOTAL_RUN]; // used for getting average

        double sumTrafficPerNodePerApp = 0d; // used for getting average

        double violationSlack = Violation.getViolationSlack();
        double trafficPerNodePerService;

        FileWriter fw = new FileWriter("/home/soroush/Desktop/FogPlan/expr4_results.txt");

        System.out.println("Tau\tTraffic\tDelay\tCost\tContainer\tViol\tDelay(MV)\tCost(MV)\tContainer(MV)\tViol(MV)\tViol_Slack=" + violationSlack + "\tThresh=" + Delay.getThresholdAverage());
        for (int Tau = MIN_CHANGE_INTERVAL; Tau <= MAX_CHANGE_INTERVAL; Tau += 5) {

            Parameters.TAU = Tau; // set the reconfiguration intervals to the current value of the reconfiguration interval
            q = Parameters.TAU / Parameters.TRAFFIC_CHANGE_INTERVAL;

            for (int i = 0; i < TOTAL_RUN; i++) {
                trafficPerNodePerService = trafficRateSetter.nextRate(); // gets the next rate
                Traffic.distributeTraffic(trafficPerNodePerService);
                sumTrafficPerNodePerApp += trafficPerNodePerService;

                Traffic.setTrafficToGlobalTraffic(MinCost);
                Traffic.setTrafficToGlobalTraffic(MinViol);
                Traffic.setTrafficToGlobalTraffic(PSObased);
                if (i % q == 0) {
                    MCDeployedServices = MinCost.run(Traffic.AGGREGATED, false);
                    MVDeployedServices = MinViol.run(Traffic.AGGREGATED, true);
                    containersDeployedPSOCRO = PSObased.run(Traffic.AGGREGATED, false);
                }
                fogContainersDeployedMinCost[i] = MCDeployedServices.getDeployedFogServices();
                cloudContainersDeployedMinCost[i] = MCDeployedServices.getDeployedCloudServices();
                delayMinViol[i] = MinViol.getAvgServiceDelay();
                costMinViol[i] = MinViol.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
                violMinViol[i] = Violation.getViolationPercentage(MinViol);

                fogContainersDeployedMinViol[i] = MVDeployedServices.getDeployedFogServices();
                cloudContainersDeployedMinViol[i] = MVDeployedServices.getDeployedCloudServices();
                delayMinCost[i] = MinCost.getAvgServiceDelay();
                costMinCost[i] = MinCost.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
                violMinCost[i] = Violation.getViolationPercentage(MinCost);

                fogContainersDeployedPSOCRO[i] = containersDeployedPSOCRO.getDeployedFogServices();
                cloudContainersDeployedPSOCRO[i] = containersDeployedPSOCRO.getDeployedCloudServices();
                delayPSOCRO[i] = PSObased.getAvgServiceDelay();
                costPSOCRO[i] = PSObased.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
                violPSOCRO[i] = Violation.getViolationPercentage(PSObased);

                System.out.println("Epoch = " + i + ", tau = " + Tau + " is finished");
            }

            System.out.print(Parameters.TAU + "\t" + ((sumTrafficPerNodePerApp * Parameters.numFogNodes * Parameters.numServices) / (TOTAL_RUN))
                    + "\t" + Statistics.findAverageOfArray(delayMinCost)
                    + "\t" + (Statistics.findAverageOfArray(costMinCost) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + Statistics.findAverageOfArray(fogContainersDeployedMinCost)
                    + "\t" + Statistics.findAverageOfArray(cloudContainersDeployedMinCost)
                    + "\t" + Statistics.findAverageOfArray(violMinCost)
                    + "\t" + Statistics.findAverageOfArray(delayMinViol)
                    + "\t" + (Statistics.findAverageOfArray(costMinViol) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + Statistics.findAverageOfArray(fogContainersDeployedMinViol)
                    + "\t" + Statistics.findAverageOfArray(cloudContainersDeployedMinViol)
                    + "\t" + Statistics.findAverageOfArray(violMinViol)
                    + "\t" + Statistics.findAverageOfArray(delayPSOCRO)
                    + "\t" + (Statistics.findAverageOfArray(costPSOCRO) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + Statistics.findAverageOfArray(fogContainersDeployedPSOCRO)
                    + "\t" + Statistics.findAverageOfArray(cloudContainersDeployedPSOCRO)
                    + "\t" + Statistics.findAverageOfArray(violPSOCRO)
            );

            fw.write(Parameters.TAU + "\t" + ((sumTrafficPerNodePerApp * Parameters.numFogNodes * Parameters.numServices) / (TOTAL_RUN))
                    + "\t" + Statistics.findAverageOfArray(delayMinCost)
                    + "\t" + (Statistics.findAverageOfArray(costMinCost) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + Statistics.findAverageOfArray(fogContainersDeployedMinCost)
                    + "\t" + Statistics.findAverageOfArray(cloudContainersDeployedMinCost)
                    + "\t" + Statistics.findAverageOfArray(violMinCost)
                    + "\t" + Statistics.findAverageOfArray(delayMinViol)
                    + "\t" + (Statistics.findAverageOfArray(costMinViol) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + Statistics.findAverageOfArray(fogContainersDeployedMinViol)
                    + "\t" + Statistics.findAverageOfArray(cloudContainersDeployedMinViol)
                    + "\t" + Statistics.findAverageOfArray(violMinViol)
                    + "\t" + Statistics.findAverageOfArray(delayPSOCRO)
                    + "\t" + (Statistics.findAverageOfArray(costPSOCRO) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + Statistics.findAverageOfArray(fogContainersDeployedPSOCRO)
                    + "\t" + Statistics.findAverageOfArray(cloudContainersDeployedPSOCRO)
                    + "\t" + Statistics.findAverageOfArray(violPSOCRO)
            );

            // prints standard deviation parameters only every 20 times
            if (Tau % 20 == 0) {
                System.out.print(
                        "\t" + Statistics.findStandardDeviationOfArray(delayMinCost)
                        + "\t" + (Statistics.findStandardDeviationOfArray(costMinCost) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                        + "\t" + Statistics.findStandardDeviationOfArray(fogContainersDeployedMinCost)
                        + "\t" + Statistics.findStandardDeviationOfArray(cloudContainersDeployedMinCost)
                        + "\t" + Statistics.findStandardDeviationOfArray(violMinCost)
                        + "\t" + Statistics.findStandardDeviationOfArray(delayMinViol)
                        + "\t" + (Statistics.findStandardDeviationOfArray(costMinViol) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                        + "\t" + Statistics.findStandardDeviationOfArray(fogContainersDeployedMinViol)
                        + "\t" + Statistics.findStandardDeviationOfArray(cloudContainersDeployedMinViol)
                        + "\t" + Statistics.findStandardDeviationOfArray(violMinViol)
                        + "\t" + Statistics.findStandardDeviationOfArray(delayPSOCRO)
                        + "\t" + (Statistics.findStandardDeviationOfArray(costPSOCRO) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                        + "\t" + Statistics.findStandardDeviationOfArray(fogContainersDeployedPSOCRO)
                        + "\t" + Statistics.findStandardDeviationOfArray(cloudContainersDeployedPSOCRO)
                        + "\t" + Statistics.findStandardDeviationOfArray(violPSOCRO)
                );

                fw.write("\t" + Statistics.findStandardDeviationOfArray(delayMinCost)
                        + "\t" + (Statistics.findStandardDeviationOfArray(costMinCost) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                        + "\t" + Statistics.findStandardDeviationOfArray(fogContainersDeployedMinCost)
                        + "\t" + Statistics.findStandardDeviationOfArray(cloudContainersDeployedMinCost)
                        + "\t" + Statistics.findStandardDeviationOfArray(violMinCost)
                        + "\t" + Statistics.findStandardDeviationOfArray(delayMinViol)
                        + "\t" + (Statistics.findStandardDeviationOfArray(costMinViol) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                        + "\t" + Statistics.findStandardDeviationOfArray(fogContainersDeployedMinViol)
                        + "\t" + Statistics.findStandardDeviationOfArray(cloudContainersDeployedMinViol)
                        + "\t" + Statistics.findStandardDeviationOfArray(violMinViol)
                        + "\t" + Statistics.findStandardDeviationOfArray(delayPSOCRO)
                        + "\t" + (Statistics.findStandardDeviationOfArray(costPSOCRO) / Parameters.TRAFFIC_CHANGE_INTERVAL)
                        + "\t" + Statistics.findStandardDeviationOfArray(fogContainersDeployedPSOCRO)
                        + "\t" + Statistics.findStandardDeviationOfArray(cloudContainersDeployedPSOCRO)
                        + "\t" + Statistics.findStandardDeviationOfArray(violPSOCRO)
                        + "\n");
            } else {
                fw.write("\n");
            }
            System.out.println();
            sumTrafficPerNodePerApp = 0d;
        }

        fw.close();
    }

}
