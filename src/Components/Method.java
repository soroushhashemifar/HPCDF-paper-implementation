package Components;

import Scheme.Parameters;
import Scheme.DeployedServices;
import Scheme.ServiceDeployMethod;
import Utilities.ArrayFiller;
import java.util.*;
import java.util.concurrent.ThreadLocalRandom;

/**
 *
 * @author Ashkan Y. and Soroush Hashemi Far
 *
 * This is the main class of the simulation, that contains different methods
 * that are proposed in the paper (e.g. optimization, greedy algorithms, all
 * cloud)
 */
public class Method {

    protected double[][] backup_lambda_in;

    DeployedServices fogStaticDeployedContainers; // the number of containers deployed when using Static Fog

    private boolean firstTimeRunDone = false; // a boolean that is used in Static Fog to keep track of the first time that the algorithm should run

    private int numFogNodes;
    private int numServices;
    private int numCloudServers;

    protected int[][] x; // x_aj
    int[][] x_backup; // backup of x
    protected int[][] xp; // x'_ak
    protected int[][] v; // v_aj

    protected double[][] d; // d_aj
    double[] Vper; // V^%_a

    protected Traffic traffic; // instance of traffic class
    protected Delay delay; // instance of delay class

    private int type; // type of the method (e.g. All Cloud vs. Min-Cost)
    protected ServiceDeployMethod scheme;

    private boolean onlyExperimental = false;
    private boolean debugMode = false;

    public int particles=20,
                epochs=700,
                gamma=3,
                initKE=100000;

    public double v_min=-2, v_max=4.;

    public double KElossRate=0.1,
            interRate=0.8;

    /**
     * Constructor of this class.
     *
     * @param scheme
     * @param numFogNodes
     * @param numServices
     * @param numCloudServers
     */
    public Method(ServiceDeployMethod scheme, int numFogNodes, int numServices, int numCloudServers) {

        traffic = new Traffic();
        delay = new Delay(this);

        this.scheme = scheme;
        type = scheme.type;
        x = ArrayFiller.convertIntegerToInt2DArray(scheme.variable.x);
        x_backup = scheme.variable.x_backup;
        xp = ArrayFiller.convertIntegerToInt2DArray(scheme.variable.xp);
        v = scheme.variable.v;

        d = scheme.variable.d;
        Vper = scheme.variable.Vper;

        this.numFogNodes = numFogNodes;
        this.numServices = numServices;
        this.numCloudServers = numCloudServers;

        backup_lambda_in = new double[numServices][numFogNodes];

//        if (numServices > 40) { // when there are large number of services, the purpose is only experimental, to show how is the performance of the system. (fog resource constraints are not checked)
//            onlyExperimental = true;
//        }

    }

    public Method clone() {
        Method new_method = new Method(this.scheme.clone(), numFogNodes, numServices, numCloudServers);

        new_method.traffic = this.traffic.clone();

        new_method.x = this.x.clone();
        new_method.x_backup = this.x_backup.clone();
        new_method.xp = this.xp.clone();

        new_method.backup_lambda_in = this.backup_lambda_in.clone();

        try {
            new_method.fogStaticDeployedContainers = this.fogStaticDeployedContainers.clone();
        } catch (Exception e){
            // Nothing
        }

        new_method.firstTimeRunDone = this.firstTimeRunDone;

        new_method.onlyExperimental = this.onlyExperimental;
        new_method.debugMode = this.debugMode;

        new_method.delay = this.delay.clone(new_method);

        return new_method;

//        return this;
    }

    /**
     * Runs the corresponding method
     *
     * @param traceType the type of the trace that is used (refer to Traffic
     * class)
     * @param isMinViol boolean showing if Min-Viol is running
     * @return returns the number of deployed fog and cloud services
     */
    public DeployedServices run(int traceType, boolean isMinViol) {
        backupAllPlacements();
        Traffic.calcArrivalRatesOfInstructions(this); // normalizes arrival rates
        delay.initialize();
        if (type == ServiceDeployMethod.ALL_CLOUD) {
            // do not change the placement
            return new DeployedServices(0, numCloudServers * numServices);
        } else if (type == ServiceDeployMethod.ALL_FOG) { // all fog is not used in the paper (only experimental)
            // do not change the placement
            return new DeployedServices(numFogNodes * numServices, 0);
        } else if (type == ServiceDeployMethod.OPTIMAL) {
            return runOptimal();
        } else if(type == ServiceDeployMethod.PSOCRO){
            return runPSObased();
        } else if (type == ServiceDeployMethod.FOG_STATIC) { // FOG_STATIC
            Traffic.backupIncomingTraffic(this);
            return runFogStatic(traceType, isMinViol);
        } else { // FOG_DYNAMIC
            return runFogDynamic(isMinViol);
        }
    }

    /**
     * Runs the optimal placement method, which will update x_aj and xp_ak
     *
     * @return returns the number of deployed fog and cloud services
     */
    private DeployedServices runOptimal() {

        Optimization.init(numServices, numFogNodes, numCloudServers);
        long numCombinations = (long) Math.pow(2, numServices * (numFogNodes + numCloudServers)); // x_aj and xp_ak
        double minimumCost = Double.MAX_VALUE, cost;
        for (long combination = 0; combination < numCombinations; combination++) { // tries all different compinations of x_aj and xp_ak
            updateDecisionVariablesAccordingToCombination(combination); // updates x, xp
            Traffic.calcArrivalRatesOfInstructions(this); // updates traffic rates
            if (Optimization.optimizationConstraintsSatisfied(x, xp, numServices, numFogNodes, numCloudServers, Parameters.L_S,
                    Parameters.L_M, Parameters.KS, Parameters.KM, Parameters.KpS, Parameters.KpM, traffic.lambdap_in)) {
                cost = getAvgCost(Parameters.TRAFFIC_CHANGE_INTERVAL);
                if (cost < minimumCost) { // updates the minimum cost
                    minimumCost = cost;
                    Optimization.updateBestDecisionVaraibles(x, xp, numServices, numFogNodes, numCloudServers);
                }
            }
        }
        Optimization.updateDecisionVaraiblesAccordingToBest(x, xp, numServices, numFogNodes, numCloudServers); // retrieve the best placement
        Traffic.calcArrivalRatesOfInstructions(this); // updates the traffic rates
        delay.initialize();
        return DeployedServices.countDeployedServices(numServices, numFogNodes, numCloudServers, x, xp);
    }

    private DeployedServices runPSObased() {
//        Random r = new Random();

        int variables = numServices * numFogNodes;
        double acceleration_coefficient = 1;

        HybridSOPsoCro swarmCro = new HybridSOPsoCro(this, particles, epochs, variables, initKE, KElossRate, interRate, gamma, v_max, v_min, numServices, numFogNodes, numCloudServers, acceleration_coefficient);
        swarmCro.run();

//        randomSearchHyperparametersSpace(50, 5, 15, 50, 200, 2, 5, variables, initKE, KElossRate, interRate, v_max, v_min);

        return DeployedServices.countDeployedServices(numServices, numFogNodes, numCloudServers, x, xp);
    }

    /**
     * Runs the Fog Static placement method, which will update x_aj and xp_ak
     *
     * @param traceType the type of the trace that is used (refer to Traffic
     * class)
     * @param isMinViol boolean showing if Min-Viol is running
     * @return returns the number of deployed fog and cloud services
     */
    private DeployedServices runFogStatic(int traceType, boolean isMinViol) {
        if (!firstTimeRunDone) { // if it is the first time
            firstTimeRunDone = true; // it does not run the algorithm after the first time
            if (traceType == Traffic.NOT_COMBINED) {
                Traffic.initializeAvgTrafficForStaticFogPlacementFirstTimePerServicePerFogNode(this);
            } else if (traceType == Traffic.AGGREGATED) {
                Traffic.initializeAvgTrafficForStaticFogPlacementFirstTimeCombined(this); // now lambda values are based on average
            } else { // if (traceType == COMBINED_APP)
                Traffic.initializeAvgTrafficForStaticFogPlacementFirstTimePerFogNode(this);
            }
            for (int a = 0; a < numServices; a++) {
                if (isMinViol) {
                    MinViol(a);
                } else {
                    MinCost(a);
                }
            }
            fogStaticDeployedContainers = DeployedServices.countDeployedServices(numServices, numFogNodes, numCloudServers, x, xp);
            Traffic.restoreIncomingTraffic(this);
            return fogStaticDeployedContainers;
        } else {
            // do not change the placement
            return fogStaticDeployedContainers;
        }
    }

    /**
     * Runs the Fog Dynamic placement method (either Min-Cost or Min-Viol),
     * which will update x_aj and xp_ak
     *
     * @param isMinViol boolean showing if Min-Viol is running
     * @return returns the number of deployed fog and cloud services
     */
    private DeployedServices runFogDynamic(boolean isMinViol) {
        for (int a = 0; a < numServices; a++) {
            if (isMinViol) {
                MinViol(a);
            } else {
                MinCost(a);
            }
        }

        return DeployedServices.countDeployedServices(numServices, numFogNodes, numCloudServers, x, xp);

    }

    /**
     * Gets the average service delay
     *
     * @return
     */
    public double getAvgServiceDelay() {
        double sumNum = 0;
        double sumDenum = 0;
        double d;
        for (int a = 0; a < Parameters.numServices; a++) {
            for (int j = 0; j < Parameters.numFogNodes; j++) {
                double sd = delay.calcServiceDelay(a, j);
                d = sd * traffic.lambda_in[a][j];
                sumNum += d;
                sumDenum += traffic.lambda_in[a][j];
            }
        }

        return sumNum / sumDenum;
    }

    /**
     * Sets the firstTimeRunDone boolean variable false
     */
    public void unsetFirstTimeBoolean() {
        firstTimeRunDone = false;
    }

    /**
     * gets the average cost for a specific time duration
     *
     * @param timeDuration the duration of the time
     */
    public double getAvgCost(double timeDuration) {
        delay.initialize();
        updateDelayAndViolation();
        return Cost.calcAverageCost(timeDuration, x, xp, x_backup, Vper, Parameters.q, traffic.lambda_in, traffic.lambdap_in, traffic.lambda_out, Parameters.L_P, Parameters.L_S, Parameters.h, this);
    }

    /**
     * Prints the current service allocation among fog nodes and cloud servers
     */
    public void printAllocation() {
        System.out.print("Fog");
        for (int j = 0; j < numFogNodes; j++) {
            System.out.print("  ");
        }
        System.out.println("Cloud");
        for (int a = 0; a < numServices; a++) {
            for (int j = 0; j < numFogNodes; j++) {
                System.out.print(x[a][j] + " ");
            }
            System.out.print("   ");
            for (int k = 0; k < numCloudServers; k++) {
                System.out.print(xp[a][k] + " ");
            }
            System.out.println("");
        }
    }

    /**
     * Runs the Min-Viol greedy algorithm for a service
     *
     * @param a the index of the service
     */
    private void MinViol(int a) {
        Violation.calcViolation(a, this);
        List<FogTrafficIndex> fogTrafficIndex = Traffic.getFogIncomingTraffic(a, false, this);
        Collections.sort(fogTrafficIndex);
        int listIndex = -1;
        int j = 0;
        while (Vper[a] > 1 - Parameters.q[a] && listIndex < numFogNodes - 1) {
            listIndex++;
            j = fogTrafficIndex.get(listIndex).getFogIndex();
            if (x[a][j] == 0 && fogHasFreeResources(j)) { // if service a is not implemented on fog node j
                // CODE: physically DEPLOY 
                x[a][j] = 1;
                placementUpdatedForService(a);
                Violation.calcViolation(a, this);
            }
        }
        boolean canRelease = true;
        listIndex = fogTrafficIndex.size();
        while (canRelease && listIndex > 0) {
            listIndex--;
            j = fogTrafficIndex.get(listIndex).getFogIndex();
            if (x[a][j] == 1) { // if service a is implemented on fog node j
                releaseServiceSafelyFromFogNodes(a, j);
                placementUpdatedForService(a);
                Violation.calcViolation(a, this);
                if (Vper[a] <= 1 - Parameters.q[a]) {
                    // CODE: physically RELEASE
                } else {
                    x[a][j] = 1;
                    placementUpdatedForService(a);
                    Violation.calcViolation(a, this);
                    canRelease = false;
                }
            }

        }
        deployOrReleaseCloudService(a);

    }

    /**
     * Runs the Min-Cost greedy algorithm for a service
     *
     * @param a the index of the service
     */
    private void MinCost(int a) {
        Violation.calcViolation(a, this);
        List<FogTrafficIndex> fogTrafficIndex = Traffic.getFogIncomingTraffic(a, false, this);
        Collections.sort(fogTrafficIndex); // sorts fog nodes based on incoming traffic
        int listIndex = -1;
        int j;
        while (listIndex < numFogNodes - 1) {
            listIndex++;
            j = fogTrafficIndex.get(listIndex).getFogIndex();
            if (x[a][j] == 0 && fogHasFreeResources(j)) { // if service a is not implemented on fog node j
                if (deployMakesSense(a, j)) {
                    // to add CODE: DEPLOY
                    x[a][j] = 1;
                    placementUpdatedForService(a);
                    Violation.calcViolation(a, this);
                }
            }
        }
        listIndex = numFogNodes;
        while (listIndex > 0) {
            listIndex--;
            j = fogTrafficIndex.get(listIndex).getFogIndex();
            if (x[a][j] == 1) { // if service a is implemented on fog node j
                if (releaseMakesSense(a, j)) {
                    // to add CODE: RELEASE
                    releaseServiceSafelyFromFogNodes(a, j);
                    placementUpdatedForService(a);
                    Violation.calcViolation(a, this);
                }
            }
        }
        deployOrReleaseCloudService(a);
    }

    /**
     * (For Min-Cost) checks if deploying a service makes sense according to
     * cost. Requirement: x[a][j] = 0;
     *
     * @param a the index of the service
     * @param j the index of the fog node
     */
    boolean deployMakesSense(int a, int j) {
        double futureCost = 0;
        double futureSavings = 0;
        double costCfc, costExtraPC, costExtraSC, costViolPerFogNode;
        // if not deploying (X[a][j] == 0) this is the cost we were paying, 
        // but now this is seen as savings
        costCfc = Cost.costCfc(Parameters.TAU, j, a, traffic.lambda_out, Parameters.h);
        costExtraPC = Cost.costExtraPC(Parameters.TAU, Parameters.h[a][j], a, Parameters.L_P, traffic.lambda_out[a][j]);
        costExtraSC = Cost.costExtraSC(Parameters.TAU, Parameters.h[a][j], a, Parameters.L_S, xp);
        double fogTrafficPercentage = calcFogTrafficPercentage(a, j);
        costViolPerFogNode = Cost.costViol(Parameters.TAU, a, j, Violation.calcVperPerNode(a, j, fogTrafficPercentage, this), Parameters.q, traffic.lambda_in);
        futureSavings = costCfc + costExtraPC + costExtraSC + costViolPerFogNode;

        // Now if we were to deploy, this is the cost we would pay
        x[a][j] = 1;
        d[a][j] = delay.calcServiceDelay(a, j);  // this is just to update the service delay

        double costDep, costPF, costSF;
        costDep = Cost.costDep(j, a, Parameters.L_S);
        costPF = Cost.costPF(Parameters.TAU, j, a, Parameters.L_P, traffic.lambda_in);
        costSF = Cost.costSF(Parameters.TAU, j, a, Parameters.L_S);
        costViolPerFogNode = Cost.costViol(Parameters.TAU, a, j, Violation.calcVperPerNode(a, j, fogTrafficPercentage, this), Parameters.q, traffic.lambda_in);
        futureCost = costDep + costPF + costSF + costViolPerFogNode;

        releaseServiceSafelyFromFogNodes(a, j); // revert this back to what it was
        d[a][j] = delay.calcServiceDelay(a, j); // revert things back to what they were
        if (futureSavings > futureCost) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * (For Min-Cost) checks if releasing a service makes sense according to
     * cost. Requirement: x[a][j] = 1;
     *
     * @param a the index of the service
     * @param j the index of the fog node
     */
    boolean releaseMakesSense(int a, int j) {
        double futureCost = 0;
        double futureSavings = 0;
        //if not releasing (X[a][j] == 1) this is the cost we were paying, 
        // but now this is seen as savings
        double costPF, costSF, costViolPerFogNode;
        costPF = Cost.costPF(Parameters.TAU, j, a, Parameters.L_P, traffic.lambda_in);
        costSF = Cost.costSF(Parameters.TAU, j, a, Parameters.L_S);
        double fogTrafficPercentage = calcFogTrafficPercentage(a, j);
        costViolPerFogNode = Cost.costViol(Parameters.TAU, a, j, Violation.calcVperPerNode(a, j, fogTrafficPercentage, this), Parameters.q, traffic.lambda_in);
        futureSavings = costPF + costSF + costViolPerFogNode;

        // Now if we were to release, this is the loss we would pay
        int k = Parameters.h[a][j];
        releaseServiceSafelyFromFogNodes(a, j);
        d[a][j] = delay.calcServiceDelay(a, j); // this is just to update the things

        double costCfc, costExtraPC, costExtraSC;
        costCfc = Cost.costCfc(Parameters.TAU, j, a, traffic.lambda_out, Parameters.h);
        costExtraPC = Cost.costExtraPC(Parameters.TAU, Parameters.h[a][j], a, Parameters.L_P, traffic.lambda_out[a][j]);
        costExtraSC = Cost.costExtraSC(Parameters.TAU, Parameters.h[a][j], a, Parameters.L_S, xp);
        costViolPerFogNode = Cost.costViol(Parameters.TAU, a, j, Violation.calcVperPerNode(a, j, fogTrafficPercentage, this), Parameters.q, traffic.lambda_in);
        futureCost = costCfc + costExtraPC + costExtraSC + costViolPerFogNode;

        x[a][j] = 1; // revert this back to what it was
        d[a][j] = delay.calcServiceDelay(a, j); // revert things back to what they were
        if (futureSavings > futureCost) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Calculates percentage of traffic in a given fog node for a particular
     * service to the total traffic to all fog nodes for that service
     *
     * @param a the index of the service
     * @param j the index of the fog node
     */
    private double calcFogTrafficPercentage(int a, int j) {
        double denum = 0;
        for (int fog = 0; fog < numFogNodes; fog++) {
            denum += traffic.lambda_in[a][fog];
        }
        return traffic.lambda_in[a][j] / denum;
    }

    /**
     * Backs up fog placement when it is called for a given service
     *
     * @param a the index of the service
     */
    private void backupFogPlacement(int a) {
        for (int j = 0; j < numFogNodes; j++) {
            x_backup[a][j] = x[a][j];
        }
    }

    /**
     * Backs up fog placement when it is called for all services
     *
     */
    public void backupAllPlacements() {
        for (int a = 0; a < numServices; a++) {
            backupFogPlacement(a);
        }
    }

    /**
     * Updates arrays x_aj and xp_ak according to the combination. (we divide
     * the combination number (its bit string) into 'numServices' chunk (e.g. if
     * numServices=5, we divide the bit string into 5 chunks). Each chunk
     * further is divided into 2 parts: form left to right, first part of the
     * bit string represents the j ('numFogNodes' bits) and the second part
     * represents the k ('numCloudServers' bits)).
     *
     * @param combination
     */
    public void updateDecisionVariablesAccordingToCombination(long combination) {
        long mask = 1;
        long temp;
        for (int a = 0; a < numServices; a++) {
            for (int k = 0; k < numCloudServers; k++) { // deploy on a cloud server, according to combination number
                temp = combination & mask;
                if (temp == 0) {
                    xp[a][k] = 0;
                } else {
                    xp[a][k] = 1;
                }
                mask = mask << 1;
            }

            for (int j = 0; j < numFogNodes; j++) { // deploy on a fog node, according to combination number
                temp = combination & mask;
                if (temp == 0) {
                    x[a][j] = 0;
                } else {
                    x[a][j] = 1;
                }
                mask = mask << 1;
            }
        }
    }

    /**
     * Determines if a fog node still has storage and memory available
     *
     * @param j the index of the fog ndoe
     */
    boolean fogHasFreeResources(int j) {
        if (onlyExperimental) {
            // since there is traffic for every (service, fog node) combination, without this boolean, fog resource capacity will limit large service deployment, and as a result, the violation will be high high. 
            return true;
        }
        double utilziedFogStorage = 0, utilziedFogMemory = 0;
        for (int a = 0; a < numServices; a++) {
            if (x[a][j] == 1) {
                utilziedFogStorage += Parameters.L_S[a];
                utilziedFogMemory += Parameters.L_M[a];
            }
        }

        if (utilziedFogStorage > Parameters.KS[j] || utilziedFogMemory > Parameters.KM[j]) {
            return false;
        }

        return true;
    }

    /**
     * This function will deploy (or release) service a on cloud services, if
     * needed, according to the traffic equations (eq. 20)
     *
     * @param a the index of the service
     */
    void deployOrReleaseCloudService(int a) {
        if (Parameters.MEASURING_RUNNING_TIME == true) {
            Traffic.calcArrivalRatesOfInstructions(this, a);
        }
        for (int k = 0; k < numCloudServers; k++) { // If incoming traffic rate to a cloud server for a particular service is 0, the service could be released to save space. On the other hand, even if there is small traffic incoming to a cloud server for a particular service, the service must not be removed from the cloud server
            if (traffic.lambdap_in[a][k] > 0) {
                xp[a][k] = 1;
//                for(Integer j : Parameters.H_inverse[a][k].elemets){
//                    releaseServiceSafelyFromFogNodes(a, j);
//                }
            } else { // lambdap_in[a][k] == 0
                xp[a][k] = 0;
//                for(Integer j : Parameters.H_inverse[a][k].elemets){
//                    traffic.lambda_out[a][j] = 0;
//                }
            }
        }
    }

    /**
     * This function will safely release a service from a fog node. (The word
     * safely refers to the case when a service is not implemented on the fog
     * node, and must be implemented on the corresponding cloud server, so that
     * the requests sent to the fog node, can be safely forwarded to that cloud
     * server)
     *
     * @param a
     * @param j
     */
    void releaseServiceSafelyFromFogNodes(int a, int j) {
        x[a][j] = 0;
        xp[a][Parameters.h[a][j]] = 1; // if there is no backup for this service in the cloud, make a backup available
    }

    /**
     * This function should be called every time the placement variables for a
     * service change
     *
     * @param a the index of the service
     */
    void placementUpdatedForService(int a) {
        if (Parameters.MEASURING_RUNNING_TIME == false) {
            Traffic.calcArrivalRatesOfInstructions(this, a);
            delay.initialize(a);
        }
    }

    /**
     * Updates average service delay, and violation as well
     */
    void updateDelayAndViolation() {
        for (int a = 0; a < numServices; a++) {
            Violation.calcViolation(a, this);
        }
    }

    double[][] getFogNodesUtilization(){
        double[][] utilization = new double[Parameters.numFogNodes][3];
        for(int i=0; i<Parameters.numFogNodes; i++) {
            double memoryUsage = 0, cpuUsage = 0, storageUsage = 0;
            int numOfDeployedServices = 0;
            for(int j=0; j<Parameters.numServices; j++) {
                if (x[j][i] == 1) {
                    memoryUsage += Parameters.L_M[j];
                    cpuUsage += Parameters.L_P[j];
                    storageUsage += Parameters.L_S[j];
                    numOfDeployedServices++;
                }
            }

            double memUtilization = numOfDeployedServices > 0 ? (memoryUsage * 1. / Parameters.KM[i]) : 0;
            double cpuUtilization = numOfDeployedServices > 0 ? (cpuUsage * 1. / (Parameters.KP[i] * 4.)) : 0;
            double storUtilization = numOfDeployedServices > 0 ? (storageUsage * 1. / Parameters.KS[i]) : 0;

            utilization[i] = new double[]{cpuUtilization, memUtilization, storUtilization};

//            System.out.println(Arrays.toString(utilization[i]));
        }

        return utilization;
    }

    public double getUtilizationFog(double impactCoeff){
        double[][] utilization = getFogNodesUtilization();

        double sum = 0;
        for(int i=0; i<numFogNodes; i++) {
            sum += impactCoeff*utilization[i][1] + (1 - impactCoeff)*utilization[i][2];
        }

        return sum / numFogNodes;
    }

    public void randomSearchHyperparametersSpace(int iterations, int minParticles, int maxParticles, int minEpochs, int maxEpochs, int minGamma, int maxGamma, int variables, int initKE, double KElossRate, double interRate, double vmax, double vmin) {
        HybridSOParticle bestModel = null;
        double minCost = Double.POSITIVE_INFINITY, minDelay = Double.POSITIVE_INFINITY;

        for (int i = 0; i < iterations; i++) {
            int numParticles = ThreadLocalRandom.current().nextInt(minParticles, maxParticles);
            int numEpochs = ThreadLocalRandom.current().nextInt(minEpochs, maxEpochs);
            int numGamma = ThreadLocalRandom.current().nextInt(minGamma, maxGamma);

            HybridSOPsoCro pso = new HybridSOPsoCro(this, numParticles, numEpochs, variables, initKE, KElossRate, interRate, numGamma, vmax, vmin, numServices, numFogNodes, numCloudServers, 1);
            HybridSOParticle res = pso.run();
            double cost = res.eval();
            double delay = res.method.getAvgServiceDelay();

            if (cost <= minCost && delay <= minDelay) {
                bestModel = res;
                minDelay = delay;
                minCost = cost;
                System.out.println("iteration=" + i + ", delay=" + minDelay + ", cost=" + minCost + ", epochs=" + numEpochs + ", gamma=" + numGamma + ", particles=" + numParticles);
            }
        }

        HybridSOParticle.applySchedule(this, bestModel, numServices, numFogNodes, 1);
    }

    public double violationDegradationWithRelease(int a, int i) {
        double oldViol = Vper[a];

        releaseServiceSafelyFromFogNodes(a, i);
        placementUpdatedForService(a);
        Violation.calcViolation(a, this);

        double newViol = Vper[a];

        // Revert changes
        x[a][i] = 1;
        placementUpdatedForService(a);
        Violation.calcViolation(a, this);

        return newViol - oldViol;
    }

    public double remainingResourcesAfterDeploy(int serviceNo, int j) {
//        double utilziedFogStorage = 0, utilziedFogMemory = 0;
//        for (int a = 0; a < numServices; a++) {
//            if (x[a][j] == 1) {
//                utilziedFogStorage += Parameters.L_S[a];
//                utilziedFogMemory += Parameters.L_M[a];
//            }
//        }
//        double availStor = Parameters.KS[j] - utilziedFogStorage;
//        double availMem = Parameters.KM[j] - utilziedFogMemory;

        x[serviceNo][j] = 1;
        placementUpdatedForService(serviceNo);
        Violation.calcViolation(serviceNo, this);

        double utilziedFogStorage = 0, utilziedFogMemory = 0;
        for (int a = 0; a < numServices; a++) {
            if (x[a][j] == 1) {
                utilziedFogStorage += Parameters.L_S[a];
                utilziedFogMemory += Parameters.L_M[a];
            }
        }
        double remainStor = Parameters.KS[j] - utilziedFogStorage;
        double remainMem = Parameters.KM[j] - utilziedFogMemory;

        releaseServiceSafelyFromFogNodes(serviceNo, j);
        placementUpdatedForService(serviceNo);
        Violation.calcViolation(serviceNo, this);

        return 2. * (remainStor * remainMem) / (remainStor + remainMem);
    }

    public double energyConsumptionFog(){
        // based on paper: A Discrete Particle Swarm Optimization approach for Energy-efficient IoT services placement over Fog infrastructures

        double energy = 0;

        // Energy consumption of computation
        for (int a = 0; a < numServices; a++){
            for (int j = 0; j < numFogNodes; j++){
                if(x[a][j] == 1){
                    energy += (Parameters.L_P[a] / Parameters.KP[j]) * (169 - 70) + 70;
                }
            }
        }

        // Energy consumption of network communications
        for (int j = 0; j < numFogNodes; j++){
            for (int k = 0; k < numCloudServers; k++){
                for (int a = 0; a < numServices; a++) {
                    if(Parameters.h[a][j] == k){
                        energy += ((Parameters.l_rq[a] / 10000) + (Parameters.dFC[j][k] + (Parameters.l_rq[a] + Parameters.l_rp[a])/Parameters.rFC[j][k])) * (169 + 318 - 70 - 145);
                    }
                }
            }
        }

        return energy;
    }
}