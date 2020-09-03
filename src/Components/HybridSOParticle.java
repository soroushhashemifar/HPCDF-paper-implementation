package Components;

import Scheme.Parameters;
import Utilities.RandomGenerator;
import com.sun.istack.internal.NotNull;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static Components.Cost.*;
import static Scheme.Parameters.*;

class HybridSOParticle {
    private Vector position;        // Current position.
    private Vector velocity;
    private Vector bestPosition;    // Personal best solution.
    private double bestEval;        // Personal best value.
    private double pe, ke;
    private int num_of_hits, PSOCoe;
    private double vmax, vmin;
    private int variables;
    Method method;
    private int numServices, numFogNodes, numCloudServers;
    private double bestViol;
    private double bestSrvDelay;

    /**
     * Construct a Particle with a random starting position.
     *
     * @param variables  the number of variables
     */
    HybridSOParticle(Method method, double vmax, double vmin, int variables, int numServices, int numFogNodes, int numCloudServers) {
        pe = Double.POSITIVE_INFINITY;
        ke = 0;
        num_of_hits = 0;
        PSOCoe = 0;
        this.method = method;
        this.numServices = numServices;
        this.numFogNodes = numFogNodes;
        this.numCloudServers = numCloudServers;
        this.vmax = vmax;
        this.vmin = vmin;
        this.variables = variables;

        position = new Vector(variables, 0);
        setRandomPosition();
        velocity = new Vector(variables, 0);
        setRandomVelocity();
        bestPosition = position.clone();
        bestEval = Double.POSITIVE_INFINITY;
        bestViol = Double.POSITIVE_INFINITY;
        bestSrvDelay = Double.POSITIVE_INFINITY;
    }

    private void setRandomPosition() {
        for(int i = 0; i < position.values.length; i++){
            position.values[i] = RandomGenerator.genIntegerRandomBetween(0, 1);
        }

        // Following code cause algorithm to stuck in all ones position!
//        for(int a = 0; a < numServices; a++){
//            for(int j = 0; j < numFogNodes; j++){
//                position.values[a*numFogNodes + j] = method.x[a][j];
//            }
//        }
    }

    private void setRandomVelocity() {
        for(int i = 0; i < velocity.values.length; i++){
            velocity.values[i] = RandomGenerator.genUniformRandomBetween(vmin, vmax);
        }
    }

    /**
     * This method convert the raw position vector to matrices used by main simulator algorithm
     *
     * @param method Method object, the main class of the simulation, created for PSO
     * @param p Final particle which position is getting used
     * @param numServices Number of services used in simulation
     * @param numFogNodes Number of fog nodes used in simulation
     */
    static void applySchedule(Method method, HybridSOParticle p, int numServices, int numFogNodes, int type){
        method.backupAllPlacements();
        int value;
        for (int i = 0; i < numServices; i++) {
            for (int j = 0; j < numFogNodes; j++) {
                if(type == 1) // To calculate for final application
                    value = (int) p.getBestPosition().values[i*numFogNodes + j];
                else // To calculate for evaluation function
                    value = (int) p.getPosition().values[i*numFogNodes + j];

                if(value == 1) {
                    method.x[i][j] = 1;
                }
                else if(value == 0) {
                    // This function will safely release a service from a fog node. (The word safely refers to the case when a service is not implemented on the fog
                    // node, and must be implemented on the corresponding cloud server, so that the requests sent to the fog node, can be safely forwarded to that cloud server)
                    method.releaseServiceSafelyFromFogNodes(i, j);
                }

                // This function should be called every time the placement variables for a service change
                method.placementUpdatedForService(i);
                // Calculate delay Violation Percentage. (Percentage of IoT requests that don't meet the delay requirement for service i (V^%_a))ˇ
                Violation.calcViolation(i, method);
            }

            // This function will deploy (or release) service a on cloud services, if needed, according to the traffic equations (eq. 20) in original paper
            method.deployOrReleaseCloudService(i);
        }
    }

    public void checkVelocityBoundaries(){
        for(int i=0; i<variables; i++){
            if(velocity.values[i] > vmax)
                velocity.values[i] = vmax;
            else if(velocity.values[i] < vmin)
                velocity.values[i] = vmin;
        }
    }

    /**
     * The evaluation of the current position.
     * @return      the evaluation
     */
    double eval () {
        applySchedule(method, this, numServices, numFogNodes, 2);
//        method.delay.initialize();
//        method.updateDelayAndViolation();

        double costPC, costPF, costSC, costSF, costCfc, costCff, costDep, costViol;

        // cost of processing in cloud
        costPC = 0;
        for (int k = 0; k < numCloudServers; k++) {
            for (int a = 0; a < numServices; a++) {
                if (method.xp[a][k] == 1) {
                    costPC += costPC(Parameters.TRAFFIC_CHANGE_INTERVAL, k, a, L_P, method.traffic.lambdap_in);
                }
            }
        }
        costPC = costPC / (numCloudServers * numServices);

        // cost of processing in fog
        costPF = 0;
        for (int j = 0; j < numFogNodes; j++) {
            for (int a = 0; a < numServices; a++) {
                if (method.x[a][j] == 1) {
                    costPF += costPF(Parameters.TRAFFIC_CHANGE_INTERVAL, j, a, L_P, method.traffic.lambda_in);
                }
            }
        }
        costPF = costPF / (numFogNodes * numServices);

        // cost of storage in cloud
        costSC = 0;
        for (int k = 0; k < numCloudServers; k++) {
            for (int a = 0; a < numServices; a++) {
                if (method.xp[a][k] == 1) {
                    costSC += costSC(Parameters.TRAFFIC_CHANGE_INTERVAL, k, a, L_S);
                }
            }
        }
        costSC = costSC / (numCloudServers * numServices);

        // cost of storage in fog
        costSF = 0;
        for (int j = 0; j < numFogNodes; j++) {
            for (int a = 0; a < numServices; a++) {
                if (method.x[a][j] == 1) {
                    costSF += costSF(Parameters.TRAFFIC_CHANGE_INTERVAL, j, a, L_S);
                }
            }
        }
        costSF = costSF / (numFogNodes * numServices);

        // cost of communication from fog to cloud
        costCfc = 0;
        for (int j = 0; j < numFogNodes; j++) {
            for (int a = 0; a < numServices; a++) {
                costCfc += costCfc(Parameters.TRAFFIC_CHANGE_INTERVAL, j, a, method.traffic.lambda_out, h);
            }
        }
        costCfc = costCfc / (numFogNodes * numServices);

        // cost of communication between fog nodes
        costCff = 0;

        // cost of container deployment
        costDep = 0;
        for (int a = 0; a < numServices; a++) {
            for (int j = 0; j < numFogNodes; j++) {
                if (method.x_backup[a][j] == 0 && method.x[a][j] == 1) {
                    costDep += costDep(j, a, L_S);
                }
            }
        }
        costDep = costDep / (numFogNodes * numServices);

        // cost of violation
        costViol = 0;
        for (int a = 0; a < numServices; a++) {
            for (int j = 0; j < numFogNodes; j++) {
                costViol += costViol(Parameters.TRAFFIC_CHANGE_INTERVAL, a, j, method.Vper[a], q, method.traffic.lambda_in);
            }
        }
        costViol = costViol / (numFogNodes * numServices);

        double costHighTraffic = 0;
        for (int a = 0; a < numServices; a++) {
            List<FogTrafficIndex> fogTrafficIndex = Traffic.getFogIncomingTraffic(a, false, method);
            Collections.sort(fogTrafficIndex);
            for (int j = 0; j < numFogNodes; j++) {
                int i = fogTrafficIndex.get(j).getFogIndex();
                costHighTraffic += ((1 - method.x[a][i])*(1./(j+1)) - method.x[a][i]*(1./(j+1))) * method.traffic.lambda_in[a][i] * 2d * Parameters.TAU;
            }
        }
        costHighTraffic = costHighTraffic / (numFogNodes * numServices);

        double costUtil = -1 * method.getUtilizationFog(0.5);

//        double costEnergy = method.energyConsumptionFog() * 0.000002 * Parameters.TAU;

        double costDelay = 0;
        for (int a = 0; a < numServices; a++) {
            for (int j = 0; j < numFogNodes; j++) {
                costDelay += method.delay.calcServiceDelay(a, j) * 0.000004 * Parameters.TAU; // 0.000004 = 0.004 for each milli-second delay
                if(method.x_backup[a][j] == 0 && method.x[a][j] == 1)
                    costDelay += method.delay.calcDeployDelay(a, j) * 0.000004;
            }
        }
        costDelay = costDelay / (numFogNodes * numServices);

        return costPC + costPF + costSC + costSF + costCff + costCfc + costViol + costDep /*+ costWrongDecision*/ + costHighTraffic + costUtil + costDelay /*+ costEnergy*/;
    }

    void updatePersonalBest() {
        pe = eval();
        double currViol = Violation.getViolationPercentage(method);
        double currSrvDelay = method.getAvgServiceDelay();
        if(pe <= bestEval && currViol <= bestViol && currSrvDelay <= bestSrvDelay){
            bestPosition = position.clone();
            bestEval = pe;
            bestViol = currViol;
            bestSrvDelay = currSrvDelay;
        }
    }

    /**
     * Update the position of a particle by adding its velocity to its position.
     */
    void updatePosition () {
        method.backupAllPlacements();

        for (int a = 0; a < this.numServices; a++) {
            // based on paper: Task Allocation for Wireless Sensor Network Using Modified Binary Particle Swarm Optimization
            for (int i = 0; i < numFogNodes; i++) {
                double rand = RandomGenerator.genUniformRandom();
                if (this.velocity.values[a * numFogNodes + i] > 0 && Math.abs(2*(sigmoid(this.velocity.values[a * numFogNodes + i]) - 0.5)) >= rand) {
                    if (method.fogHasFreeResources(i)) {
                        this.position.values[a * numFogNodes + i] = 1;
                        method.x[a][i] = 1;
                        method.placementUpdatedForService(a);
                        Violation.calcViolation(a, method);
                    }
                }
                else if (this.velocity.values[a * numFogNodes + i] <= 0 && Math.abs(2*(sigmoid(this.velocity.values[a * numFogNodes + i]) - 0.5)) >= rand) {
//                    if (method.violationDegradationWithRelease(a, i)>0) {
                        this.position.values[a * numFogNodes + i] = 0;
                        method.releaseServiceSafelyFromFogNodes(a, i);
                        method.placementUpdatedForService(a);
                        Violation.calcViolation(a, method);
//                    }
                }
            }

            method.deployOrReleaseCloudService(a);
        }

//        System.out.println("fog befor: " + Arrays.deepToString(method.x));
//        System.out.println("cloud before: " + Arrays.deepToString(method.xp));
    }

    private double sigmoid(double value){
        return 1. / (1. + Math.exp(-1 * value));
    }

    int bestFitPlacement(int serviceNo){
        int bestFogNode = -1;
        double minRemain = Double.POSITIVE_INFINITY;
        for(int j=0; j<numFogNodes; j++) {
            if(method.x[serviceNo][j] == 1 || !method.fogHasFreeResources(j))
                continue;

            double remainRes = method.remainingResourcesAfterDeploy(serviceNo, j);
            if(remainRes >= 0 && remainRes < minRemain){
                bestFogNode = j;
                minRemain = remainRes;
            }
        }

        return bestFogNode;
    }

    void swapPosition(){
        method.backupAllPlacements();
        int index, serviceNo, fogNodeNo, index2, serviceNo2, fogNodeNo2;

        for(int i=0; i<variables/4; i++) {
            while(true) {
                index = RandomGenerator.genIntegerRandomBetween(0, variables - 1);
                serviceNo = (int) Math.floor(index * 1. / numFogNodes);
                fogNodeNo = index - numFogNodes * serviceNo;

                index2 = RandomGenerator.genIntegerRandomBetween(0, variables - 1);
                serviceNo2 = (int) Math.floor(index2 * 1. / numFogNodes);
                fogNodeNo2 = index2 - numFogNodes * serviceNo2;

                if(index == index2)
                    continue;

                if (position.values[index] == 0 && position.values[index2] == 1) {
//                    if (method.violationDegradationWithRelease(serviceNo2, fogNodeNo2)>0) {
                        position.values[index2] = 0;
                        method.releaseServiceSafelyFromFogNodes(serviceNo2, fogNodeNo2);
                        method.placementUpdatedForService(serviceNo2);
                        Violation.calcViolation(serviceNo2, method);
//                    }
//                    else
//                        continue;

                    // based on paper: Chemical reaction optimization for virtual machine placement in cloud computing
                    if (method.fogHasFreeResources(fogNodeNo)) {
                        position.values[index] = 1;
                        method.x[serviceNo][fogNodeNo] = 1;
                    }
                    else{
                        int alternativeFogNodeNo = bestFitPlacement(serviceNo);
                        if(alternativeFogNodeNo != -1){
                            position.values[index] = 1;
                            method.x[serviceNo][fogNodeNo] = 1;
                        }
                        else{
//                            System.out.println("not feasible");
//                            position.values[index2] = 1;
//                            method.x[serviceNo2][fogNodeNo2] = 1;
//                            method.placementUpdatedForService(serviceNo2);
//                            Violation.calcViolation(serviceNo2, method);
                            continue;
                        }
                    }

                    method.placementUpdatedForService(serviceNo);
                    Violation.calcViolation(serviceNo, method);
                    break;
                }
                else if (position.values[index] == 1 && position.values[index2] == 0) {
//                    if (method.violationDegradationWithRelease(serviceNo, fogNodeNo)>0) {
                        position.values[index] = 0;
                        method.releaseServiceSafelyFromFogNodes(serviceNo, fogNodeNo);
                        method.placementUpdatedForService(serviceNo);
                        Violation.calcViolation(serviceNo, method);
//                    }
//                    else
//                        continue;

                    if (method.fogHasFreeResources(fogNodeNo2)) {
                        position.values[index2] = 1;
                        method.x[serviceNo2][fogNodeNo2] = 1;
                    }
                    else{
                        int alternativeFogNodeNo2 = bestFitPlacement(serviceNo2);
                        if(alternativeFogNodeNo2 != -1){
                            position.values[index2] = 1;
                            method.x[serviceNo2][fogNodeNo2] = 1;
                        }
                        else{
//                            System.out.println("not feasible");
//                            position.values[index] = 1;
//                            method.x[serviceNo][fogNodeNo] = 1;
//                            method.placementUpdatedForService(serviceNo);
//                            Violation.calcViolation(serviceNo, method);
                            continue;
                        }
                    }

                    method.placementUpdatedForService(serviceNo2);
                    Violation.calcViolation(serviceNo2, method);
                    break;
                }
                else if (position.values[index] == position.values[index2])
                    break;
            }

            method.deployOrReleaseCloudService(serviceNo);
            method.deployOrReleaseCloudService(serviceNo2);
        }
    }

    void swapPosition(@NotNull Vector v1, @NotNull Vector v2){
        method.backupAllPlacements();
        int index, serviceNo, fogNodeNo, index2, serviceNo2, fogNodeNo2;
        for (int i=0; i<variables/4; i++) {
            while(true) {
                index = RandomGenerator.genIntegerRandomBetween(0, variables - 1);
                serviceNo = (int) Math.floor(index * 1. / numFogNodes);
                fogNodeNo = index - numFogNodes * serviceNo;

                if (v1.values[index] == 0 && position.values[index] == 1) {
//                    if (method.violationDegradationWithRelease(serviceNo, fogNodeNo)>0) {
                        position.values[index] = v1.values[index];
                        method.releaseServiceSafelyFromFogNodes(serviceNo, fogNodeNo);
//                    }
//                    else
//                        continue;
//
                    break;
                }
                else if (v1.values[index] == 1 && position.values[index] == 0) {
                    if (method.fogHasFreeResources(fogNodeNo)) {
                        position.values[index] = 1;
                        method.x[serviceNo][fogNodeNo] = 1;
                    }
                    else{
                        int alternativeFogNodeNo = bestFitPlacement(serviceNo);
                        if(alternativeFogNodeNo != -1){
                            position.values[index] = 1;
                            method.x[serviceNo][fogNodeNo] = 1;
                        }
                        else{
//                            System.out.println("not feasible");
                            continue;
                        }
                    }

                    break;
                }
                else if (v1.values[index] == position.values[index])
                    break;
            }

            method.placementUpdatedForService(serviceNo);
            Violation.calcViolation(serviceNo, method);
            method.deployOrReleaseCloudService(serviceNo);

            while(true) {
                index2 = RandomGenerator.genIntegerRandomBetween(0, variables - 1);
                serviceNo2 = (int) Math.floor(index2 * 1. / numFogNodes);
                fogNodeNo2 = index2 - numFogNodes * serviceNo2;

                if(index == index2)
                    continue;

                if (v2.values[index2] == 0 && position.values[index2] == 1) {
//                    if (method.violationDegradationWithRelease(serviceNo2, fogNodeNo2)>0) {
                        position.values[index2] = v2.values[index2];
                        method.releaseServiceSafelyFromFogNodes(serviceNo2, fogNodeNo2);
//                    }
//                    else
//                        continue;

                    break;
                }
                else if (v2.values[index2] == 1 && position.values[index2] == 0) {
                    if (method.fogHasFreeResources(fogNodeNo2)) {
                        position.values[index2] = 1;
                        method.x[serviceNo2][fogNodeNo2] = 1;
                    }
                    else{
                        int alternativeFogNodeNo2 = bestFitPlacement(serviceNo2);
                        if(alternativeFogNodeNo2 != -1){
                            position.values[index2] = 1;
                            method.x[serviceNo2][fogNodeNo2] = 1;
                        }
                        else{
//                            System.out.println("not feasible");
                            continue;
                        }
                    }

                    break;
                }
                else if (v2.values[index2] == position.values[index2])
                    break;
            }

            method.placementUpdatedForService(serviceNo2);
            Violation.calcViolation(serviceNo2, method);
            method.deployOrReleaseCloudService(serviceNo2);
        }
    }

    void setBestPosition(@NotNull Vector bestPosition) { this.bestPosition = bestPosition.clone(); }

    double getPe() {
        return pe;
    }

    void setPe(double pe) {
        this.pe = pe;
    }

    double getKe() {
        return ke;
    }

    void setKe(double ke) {
        this.ke = ke;
    }

    void incrementNumOfHits() {
        num_of_hits += 1;
    }

    int getPSOCoe() {
        return this.PSOCoe;
    }

    void resetPSOCoe() {
        this.PSOCoe = 0;
    }

    void incrementPSOCoe() {
        this.PSOCoe += 1;
    }

    void setBestEval(double bestEval) {
        this.bestEval = bestEval;
    }

    /**
     * Get a copy of the position of the particle.
     * @return  the x position
     */
    Vector getPosition() {
        return position.clone();
    }

    /**
     * Get a copy of the velocity of the particle.
     * @return  the velocity
     */
    Vector getVelocity () {
        return velocity.clone();
    }

    /**
     * Get a copy of the personal best solution.
     * @return  the best position
     */
    Vector getBestPosition() {
        return bestPosition.clone();
    }

    /**
     * Get the value of the personal best solution.
     * @return  the evaluation
     */
    double getBestEval () {
        return bestEval;
    }

    public double getBestViol() {
        return bestViol;
    }

    public double getBestSrvDelay() {
        return bestSrvDelay;
    }

    /**
     * Set the velocity of the particle.
     * @param velocity  the new velocity
     */
    void setVelocity (Vector velocity) {
        this.velocity = velocity.clone();
    }

    void copy(@NotNull HybridSOParticle p){
        position = p.position.clone();
        velocity = p.velocity.clone();
        bestPosition = p.bestPosition.clone();
        bestEval = p.bestEval;
        pe = p.pe;
        ke = p.ke;
        num_of_hits = p.num_of_hits;
        PSOCoe = p.PSOCoe;
        method = p.method.clone();
    }

//    boolean position_equals(HybridSOParticle particle){
//        for(int i=0; i<variables; i++)
//            if(particle.position.values[i] != this.position.values[i])
//                return false;
//
//        return true;
//    }
//
//    void decPositionUpdate() {
//        ArrayList<Integer> list = new ArrayList<>();
//        for (int i=0; i<variables; i++) {
//            list.add(i);
//        }
//        Collections.shuffle(list);
//
//        position.values[list.get(0)] = RandomGenerator.genIntegerRandomBetween(0, 1);
//    }
//
//    void synPositionUpdate(HybridSOParticle particle) {
//        ArrayList<Integer> list = new ArrayList<>();
//        for (int i=0; i<variables; i++) {
//            list.add(i);
//        }
//        Collections.shuffle(list);
//
//        if(RandomGenerator.genUniformRandom() < 0.5)
//            position.values[list.get(0)] = particle.position.values[list.get(0)];
//        else
//            position.values[list.get(1)] = particle.position.values[list.get(1)];
//    }
}
