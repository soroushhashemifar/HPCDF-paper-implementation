package Run;

import Components.Method;
import Components.Traffic;
import Components.Violation;
import Scheme.DeployedServices;
import Scheme.Parameters;
import Scheme.ServiceDeployMethod;
import Trace.AggregatedTraceReader;
import java.io.FileWriter;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;

/**
 *
 * @author Ashkan Y.
 *
 * This is the main class for experiment 1
 */
public class MainExperiment1 {

    private static int TOTAL_RUN;

    private static int index = 0;

    public static boolean printCost = false;

    private static int TAU = 15 * 60; // time interval between run of the method (s)
    private static int TRAFFIC_CHANGE_INTERVAL = 15 * 60; // time interval between run of the method (sec)

    public static void main(String[] args) throws IOException {
        // in each experiment, these parameters may vary
        Parameters.numCloudServers = 3;
        Parameters.numFogNodes = 10;
        Parameters.numServices = 40;
        Traffic.TRAFFIC_ENLARGE_FACTOR = 1;
        Parameters.initialize();

        ArrayList<Double> traceList = AggregatedTraceReader.readTrafficFromFile(); // read the traffic

        TOTAL_RUN = traceList.size();

        Parameters.TAU = TAU;
        Parameters.TRAFFIC_CHANGE_INTERVAL = TRAFFIC_CHANGE_INTERVAL;
        int q = Parameters.TAU / Parameters.TRAFFIC_CHANGE_INTERVAL; // the number of times that traffic changes between each run of the method

        Method AllCloud = new Method(new ServiceDeployMethod(ServiceDeployMethod.ALL_CLOUD), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method AllFog = new Method(new ServiceDeployMethod(ServiceDeployMethod.ALL_FOG), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method FogStatic = new Method(new ServiceDeployMethod(ServiceDeployMethod.FOG_STATIC, AggregatedTraceReader.averageTrafficTrace), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method MinCost = new Method(new ServiceDeployMethod(ServiceDeployMethod.FOG_DYNAMIC), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method FogStaticViolation = new Method(new ServiceDeployMethod(ServiceDeployMethod.FOG_STATIC, AggregatedTraceReader.averageTrafficTrace), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method MinViol = new Method(new ServiceDeployMethod(ServiceDeployMethod.FOG_DYNAMIC), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        Method PSObased = new Method(new ServiceDeployMethod(ServiceDeployMethod.PSOCRO), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);

        DeployedServices containersDeployedAllCloud = null;
        DeployedServices containersDeployedAllFog = null;
        DeployedServices containersDeployedFogStatic = null;
        DeployedServices containersDeployedMinCost = null;
        DeployedServices containersDeployedFogStaticViolation = null;
        DeployedServices containersDeployedMinViol = null;
        DeployedServices containersDeployedPSOCRO = null;

        double delayAllCloud = 0;
        double delayAllFog = 0;
        double delayFogStatic = 0;
        double delayMinCost = 0;
        double delayFogStaticViolation = 0;
        double delayMinViol = 0;
        double delayPSOCRO= 0;

        double costAllCloud = 0;
        double costAllFog = 0;
        double costFogStatic = 0;
        double costMinCost = 0;
        double costFogStaticViolation = 0;
        double costMinViol = 0;
        double costPSOCRO = 0;

        double violAllCloud = 0;
        double violAllFog = 0;
        double violFogStatic = 0;
        double violMinCost = 0;
        double violFogStaticViolation = 0;
        double violMinViol = 0;
        double violPSOCRO = 0;

        double timeAllCloud = 0;
        double timeAllFog = 0;
        double timeFogStatic = 0;
        double timeMinCost = 0;
        double timeFogStaticViolation = 0;
        double timeMinViol = 0;
        double timePSOCRO = 0;

        double violationSlack = Violation.getViolationSlack();
        Double trafficPerNodePerService;

        FileWriter fw = new FileWriter("/home/soroush/Desktop/FogPlan/expr1_results.txt");
//        FileWriter fw_times = new FileWriter("/home/soroush/Desktop/FogPlan/expr1_times.txt");

        System.out.println("No.\tTraffic\tD(AC)\tD(AF)\tD(FS)\tD(MC)\tD(FSV)\tD(MV)\tD(OP)\tC(AC)\tC(AF)\tC(FS)\tC(MC)\tC(FSV)\tC(MV)\tC(OP)\tCNT(AC)\tCNT(AF)\tCNT(FS)\tCNT(MC)\tCNT(FSV)\tCNT(MV)\tCNT(OP)\tV(AC)\tV(AF)\tV(FS)\tV(MC)\tV(FSV)\tV(MV)\tV(OP)\tVS=" + violationSlack);
        for (int i = 0; i < TOTAL_RUN; i++) {
            trafficPerNodePerService = nextRate(traceList); // get the next traffic rate
            Traffic.distributeTraffic(trafficPerNodePerService);

            Traffic.setTrafficToGlobalTraffic(AllCloud);
//            Instant start = Instant.now();
            containersDeployedAllCloud = AllCloud.run(Traffic.AGGREGATED, false);
//            timeAllCloud = Duration.between(start, Instant.now()).toMillis();  //in millis
            delayAllCloud = AllCloud.getAvgServiceDelay();
            costAllCloud = AllCloud.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violAllCloud = Violation.getViolationPercentage(AllCloud);

            Traffic.setTrafficToGlobalTraffic(AllFog);
//            start = Instant.now();
            containersDeployedAllFog = AllFog.run(Traffic.AGGREGATED, false);
//            timeAllFog = Duration.between(start, Instant.now()).toMillis();  //in millis
            delayAllFog = AllFog.getAvgServiceDelay();
            costAllFog = AllFog.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violAllFog = Violation.getViolationPercentage(AllFog);

            Traffic.setTrafficToGlobalTraffic(FogStatic);
//            start = Instant.now();
            containersDeployedFogStatic = FogStatic.run(Traffic.AGGREGATED, false);
//            timeFogStatic = Duration.between(start, Instant.now()).toMillis();  //in millis
            delayFogStatic = FogStatic.getAvgServiceDelay();
            costFogStatic = FogStatic.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violFogStatic = Violation.getViolationPercentage(FogStatic);

            Traffic.setTrafficToGlobalTraffic(MinCost);
//            start = Instant.now();
            if (i % q == 0) {
                containersDeployedMinCost = MinCost.run(Traffic.AGGREGATED, false);
            }
//            timeMinCost = Duration.between(start, Instant.now()).toMillis();  //in millis
            delayMinCost = MinCost.getAvgServiceDelay();
            costMinCost = MinCost.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violMinCost = Violation.getViolationPercentage(MinCost);

            Traffic.setTrafficToGlobalTraffic(FogStaticViolation);
//            start = Instant.now();
            containersDeployedFogStaticViolation = FogStaticViolation.run(Traffic.AGGREGATED, true);
//            timeFogStaticViolation = Duration.between(start, Instant.now()).toMillis();  //in millis
            delayFogStaticViolation = FogStaticViolation.getAvgServiceDelay();
            costFogStaticViolation = FogStaticViolation.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violFogStaticViolation = Violation.getViolationPercentage(FogStaticViolation);

            Traffic.setTrafficToGlobalTraffic(MinViol);
//            start = Instant.now();
            if (i % q == 0) {
                containersDeployedMinViol = MinViol.run(Traffic.AGGREGATED, true);
            }
//            timeMinViol = Duration.between(start, Instant.now()).toMillis();  //in millis
            delayMinViol = MinViol.getAvgServiceDelay();
            costMinViol = MinViol.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violMinViol = Violation.getViolationPercentage(MinViol);

            Traffic.setTrafficToGlobalTraffic(PSObased);
//            start = Instant.now();
            if (i % q == 0)
                containersDeployedPSOCRO = PSObased.run(Traffic.AGGREGATED, false);
//            timePSOCRO = Duration.between(start, Instant.now()).toMillis();  //in millis
            delayPSOCRO = PSObased.getAvgServiceDelay();
            costPSOCRO = PSObased.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violPSOCRO = Violation.getViolationPercentage(PSObased);

            System.out.println(i+1 + "\t" + (trafficPerNodePerService * Parameters.numFogNodes * Parameters.numServices) + "\t" + delayAllCloud + "\t" + delayAllFog + "\t" + delayFogStatic + "\t" + delayMinCost + "\t" + delayFogStaticViolation + "\t" + delayMinViol + "\t" + delayPSOCRO
                    + "\t" + (costAllCloud / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costAllFog / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costFogStatic / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costMinCost / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costFogStaticViolation / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costMinViol / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costPSOCRO / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + containersDeployedAllCloud.getDeployedFogServices() + "\t" + containersDeployedAllFog.getDeployedFogServices() + "\t" + containersDeployedFogStatic.getDeployedFogServices() + "\t" + containersDeployedMinCost.getDeployedFogServices() + "\t" + containersDeployedFogStaticViolation.getDeployedFogServices() + "\t" + containersDeployedMinViol.getDeployedFogServices() + "\t" + containersDeployedPSOCRO.getDeployedFogServices()
                    + "\t" + containersDeployedAllCloud.getDeployedCloudServices() + "\t" + containersDeployedAllFog.getDeployedCloudServices() + "\t" + containersDeployedFogStatic.getDeployedCloudServices() + "\t" + containersDeployedMinCost.getDeployedCloudServices() + "\t" + containersDeployedFogStaticViolation.getDeployedCloudServices() + "\t" + containersDeployedMinViol.getDeployedCloudServices() + "\t" + containersDeployedPSOCRO.getDeployedCloudServices()
                    + "\t" + violAllCloud + "\t" + violAllFog + "\t" + violFogStatic + "\t" + violMinCost + "\t" + violFogStaticViolation + "\t" + violMinViol + "\t" + violPSOCRO
            );

            fw.write(i+1 + "\t" + (trafficPerNodePerService * Parameters.numFogNodes * Parameters.numServices) + "\t" + delayAllCloud + "\t" + delayAllFog + "\t" + delayFogStatic + "\t" + delayMinCost + "\t" + delayFogStaticViolation + "\t" + delayMinViol + "\t" + delayPSOCRO
                    + "\t" + (costAllCloud / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costAllFog / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costFogStatic / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costMinCost / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costFogStaticViolation / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costMinViol / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" + (costPSOCRO / Parameters.TRAFFIC_CHANGE_INTERVAL)
                    + "\t" + containersDeployedAllCloud.getDeployedFogServices() + "\t" + containersDeployedAllFog.getDeployedFogServices() + "\t" + containersDeployedFogStatic.getDeployedFogServices() + "\t" + containersDeployedMinCost.getDeployedFogServices() + "\t" + containersDeployedFogStaticViolation.getDeployedFogServices() + "\t" + containersDeployedMinViol.getDeployedFogServices() + "\t" + containersDeployedPSOCRO.getDeployedFogServices()
                    + "\t" + containersDeployedAllCloud.getDeployedCloudServices() + "\t" + containersDeployedAllFog.getDeployedCloudServices() + "\t" + containersDeployedFogStatic.getDeployedCloudServices() + "\t" + containersDeployedMinCost.getDeployedCloudServices() + "\t" + containersDeployedFogStaticViolation.getDeployedCloudServices() + "\t" + containersDeployedMinViol.getDeployedCloudServices() + "\t" + containersDeployedPSOCRO.getDeployedCloudServices()
                    + "\t" + violAllCloud + "\t" + violAllFog + "\t" + violFogStatic + "\t" + violMinCost + "\t" + violFogStaticViolation + "\t" + violMinViol + "\t" + violPSOCRO
                    + "\n"
            );

//            fw_times.write(i+1 + "\t" + timeAllCloud + "\t" + timeAllFog + "\t" + timeFogStatic + "\t" + timeMinCost + "\t" + timeFogStaticViolation + "\t" + timeMinViol + "\t" + timePSOCRO + "\n");
        }

        fw.close();
//        fw_times.close();
    }

    /**
     * Gets the next traffic rate from the trace
     *
     * @param traceList the trace
     * @return returns the next traffic rate from the trace
     */
    private static Double nextRate(ArrayList<Double> traceList) {
        return traceList.get(index++);
    }

}
