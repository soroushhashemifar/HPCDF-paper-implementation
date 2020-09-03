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
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadLocalRandom;

/**
 *
 * @author Ashkan Y.
 *
 * This is the main class for experiment 1
 */
public class MainExperiment1new {

    private static int TOTAL_RUN;
    private static int index = 0;
    private static int TAU = 15 * 60; // time interval between run of the method (s)
    private static int TRAFFIC_CHANGE_INTERVAL = 15 * 60; // time interval between run of the method (sec)

    public static void main(String[] args) throws IOException {
        Parameters.numCloudServers = 3;
        Parameters.numFogNodes = 10;
        Parameters.numServices = 40;
        Traffic.TRAFFIC_ENLARGE_FACTOR = 1;
        Parameters.initialize();

        ArrayList<Double> traceList = AggregatedTraceReader.readTrafficFromFile(); // read the traffic
        TOTAL_RUN = traceList.size();

        Parameters.TAU = TAU;
        Parameters.TRAFFIC_CHANGE_INTERVAL = TRAFFIC_CHANGE_INTERVAL;

        ExecutorService tpes = Executors.newFixedThreadPool(4);

        System.out.println("No\tEpochs\tParticles\tGamma\tDelay\tCost\tFS\tCS\tViol");
        for (int i = 0; i < 500; i++) {
            int numParticles = ThreadLocalRandom.current().nextInt(5, 50);
            int numEpochs = 200; //ThreadLocalRandom.current().nextInt(100, 1000);
            int numGamma = ThreadLocalRandom.current().nextInt(2, 10);

            tpes.execute(new WorkerThread(i, numEpochs, numParticles, numGamma, TOTAL_RUN, new ArrayList<>(traceList)));
        }

        tpes.shutdown();
    }

    public static Double arrayAverage(double[] array) {
        double sum = 0;
        for(double d : array) {
            sum += d;
        }

        return sum * 1. / array.length;
    }

    public static Double arrayAverage(int[] array) {
        int sum = 0;
        for(int d : array) {
            sum += d;
        }

        return sum * 1. / array.length;
    }
}

class WorkerThread implements Runnable {
    private int epochs, numOfParticles, gamma, TOTAL_RUN;
    private int index = 0;
    ArrayList<Double> traceList;
    private int threadIndex;

    WorkerThread(int index, int epochs, int numOfParticles, int gamma, int TOTAL_RUN, ArrayList<Double> traceList) {
        this.epochs = epochs;
        this.numOfParticles = numOfParticles;
        this.gamma = gamma;
        this.TOTAL_RUN = TOTAL_RUN;
        this.traceList = traceList;
        this.threadIndex = index;
    }

    public void run() {
        System.out.println("Thread #" + threadIndex + " has been created for epoch " + epochs + ", particles "+ numOfParticles +", gamma " + gamma);

        Method PSObased = new Method(new ServiceDeployMethod(ServiceDeployMethod.PSOCRO), Parameters.numFogNodes, Parameters.numServices, Parameters.numCloudServers);
        PSObased.epochs = epochs;
        PSObased.particles = numOfParticles;
        PSObased.gamma = gamma;

        DeployedServices containersDeployedPSOCRO;
        int[] DeployedFogServices = new int[TOTAL_RUN];
        int[] DeployedCloudServices = new int[TOTAL_RUN];
        double[] delayPSOCRO = new double[TOTAL_RUN];
        double[] costPSOCRO = new double[TOTAL_RUN];
        double[] violPSOCRO = new double[TOTAL_RUN];

        Double trafficPerNodePerService;
        for (int i = 0; i < TOTAL_RUN; i++) {
            trafficPerNodePerService = nextRate(traceList); // get the next traffic rate
            Traffic.distributeTraffic(trafficPerNodePerService);

            Traffic.setTrafficToGlobalTraffic(PSObased);
            containersDeployedPSOCRO = PSObased.run(Traffic.AGGREGATED, false);
            DeployedFogServices[i] = containersDeployedPSOCRO.getDeployedFogServices();
            DeployedCloudServices[i] = containersDeployedPSOCRO.getDeployedCloudServices();
            delayPSOCRO[i] = PSObased.getAvgServiceDelay();
            costPSOCRO[i] = PSObased.getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
            violPSOCRO[i] = Violation.getViolationPercentage(PSObased);
        }

        System.out.println(threadIndex + "\t" + epochs + "\t" + numOfParticles + "\t" + gamma + "\t" +
                MainExperiment1new.arrayAverage(delayPSOCRO) + "\t" +
                (MainExperiment1new.arrayAverage(costPSOCRO) / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" +
                MainExperiment1new.arrayAverage(DeployedFogServices) + "\t" +
                MainExperiment1new.arrayAverage(DeployedCloudServices) + "\t" +
                MainExperiment1new.arrayAverage(violPSOCRO));

        FileWriter fw;
        try {
            fw = new FileWriter("/home/soroush/Desktop/FogPlan/expr1new_results.txt", true);

            fw.write(epochs + "\t" + numOfParticles + "\t" + gamma + "\t" +
                    MainExperiment1new.arrayAverage(delayPSOCRO) + "\t" +
                    (MainExperiment1new.arrayAverage(costPSOCRO) / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" +
                    MainExperiment1new.arrayAverage(DeployedFogServices) + "\t" +
                    MainExperiment1new.arrayAverage(DeployedCloudServices) + "\t" +
                    MainExperiment1new.arrayAverage(violPSOCRO) + "\n");

            fw.close();

        } catch (IOException e) {

            try {
                Thread.sleep(500);
                fw = new FileWriter("/home/soroush/Desktop/FogPlan/expr1new_results.txt", true);
                fw.write(epochs + "\t" + numOfParticles + "\t" + gamma + "\t" +
                        MainExperiment1new.arrayAverage(delayPSOCRO) + "\t" +
                        (MainExperiment1new.arrayAverage(costPSOCRO) / Parameters.TRAFFIC_CHANGE_INTERVAL) + "\t" +
                        MainExperiment1new.arrayAverage(DeployedFogServices) + "\t" +
                        MainExperiment1new.arrayAverage(DeployedCloudServices) + "\t" +
                        MainExperiment1new.arrayAverage(violPSOCRO) + "\n");

                fw.close();
            } catch (Exception ex) {
                System.out.println("Could not write results for epoch " + epochs + ", particles "+ numOfParticles +", gamma " + gamma);
            }
        }
    }

    private Double nextRate(ArrayList<Double> traceList) {
        return traceList.get(index++);
    }
}