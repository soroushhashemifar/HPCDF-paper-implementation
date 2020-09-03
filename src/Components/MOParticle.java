package Components;

import Scheme.Parameters;
import java.util.Arrays;
import java.util.Random;

/**
 *
 * @author Soroush Hashemi Far
 *
 * This is the class of Particle used in Multi-Objective Particle Swarm Optimization algorithm.
 */
public class MOParticle {

    private int numberOfVariables, numberOfObjectives;
    private double pos_lower_bound, pos_upper_bound, vmin, vmax;
    private double[] cost;
    private Vector pbest;
    private double[] cbest;
    private Vector position, velocity;
    Method method;
    private int numServices, numFogNodes, numCloudServers;

    /**
     * Initializes the Particle instance
     *
     * @param method Method object, the main class of the simulation, created for PSO
     * @param numberOfObjectives Number of objectives for PSO
     * @param numberOfVariables Number of dimensions of the search space
     * @param pos_lower_bound Minimum value of each element of position vector
     * @param pos_upper_bound Maximum value of each element of position vector
     * @param vmax Maximum value of each element of velocity vector
     * @param vmin Minimum value of each element of velocity vector
     * @param numServices Number of services used in simulation
     * @param numFogNodes Number of fog nodes used in simulation
     * @param numCloudServers Number of cloud servers used in simulation
     */
    MOParticle(Method method, int numberOfObjectives, double pos_lower_bound, double pos_upper_bound, double vmin, double vmax, int numberOfVariables, int numServices, int numFogNodes, int numCloudServers) {
        this.pos_lower_bound = pos_lower_bound;
        this.pos_upper_bound = pos_upper_bound;
        this.vmin = vmin;
        this.vmax = vmax;
        this.numberOfVariables = numberOfVariables;
        this.numberOfObjectives = numberOfObjectives;
        this.cost = new double[numberOfObjectives];
        this.cbest = new double[numberOfObjectives];
        this.method = method;
        this.numServices = numServices;
        this.numFogNodes = numFogNodes;
        this.numCloudServers = numCloudServers;
        for(int i=0; i<this.numberOfObjectives; i++) { // Initializes cost and personal best cost with +infinite
            this.cost[i] = Double.MAX_VALUE;
            this.cbest[i] = Double.MAX_VALUE;
        }

        // Create position and velocity vectors with initial values equal to zero
        this.position = new Vector(this.numberOfVariables, 0);
        this.velocity = new Vector(this.numberOfVariables, 0);
        Random r = new Random();
        for(int i=0; i<this.numberOfVariables; i++) { // Refill position and velocity vectors with random numbers
            this.position.values[i] = r.nextInt(2);
            this.velocity.values[i] = this.vmin + (this.vmax - this.vmin) * r.nextDouble();
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
    static void applySchedule(Method method, MOParticle p, int numServices, int numFogNodes){
        for (int i = 0; i < numServices; i++) {
            for (int j = 0; j < numFogNodes; j++) {
                int value = (int) p.getPbest()[i*numFogNodes + j];

                if(value == 1) {
                    method.x[i][j] = 1;
                }
                else {
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

    /**
     * This method is the main cost function of PSO algorithm to calculate cost of current position
     * Keep this total cost value in a vector embedded in each particle
     */
    void evaluate_cost_function() {
        double[] costF = new double[this.numberOfObjectives];
        Arrays.fill(costF, 0);

        applySchedule(method, this, numServices, numFogNodes);

        // Cost of processing and storage in cloud
        for (int k = 0; k < numCloudServers; k++) {
            for (int a = 0; a < numServices; a++) {
                if (method.xp[a][k] == 1) {
                    costF[0] += Cost.costPC(Parameters.TRAFFIC_CHANGE_INTERVAL, k, a, Parameters.L_P, method.traffic.lambdap_in) / (numCloudServers * numServices);
                    costF[1] += Cost.costSC(Parameters.TRAFFIC_CHANGE_INTERVAL, k, a, Parameters.L_S) / (numCloudServers * numServices);
                }
            }
        }

        // Cost of processing and storage in fog and cost of communication from fog to cloud
        for (int j = 0; j < numFogNodes; j++) {
            for (int a = 0; a < numServices; a++) {
                if (method.x[a][j] == 1) {
                    costF[2] += Cost.costPF(Parameters.TRAFFIC_CHANGE_INTERVAL, j, a, Parameters.L_P, method.traffic.lambda_in) / (numFogNodes * numServices);
                    costF[3] += Cost.costSF(Parameters.TRAFFIC_CHANGE_INTERVAL, j, a, Parameters.L_S) / (numFogNodes * numServices);
                    costF[4] += Cost.costCfc(Parameters.TRAFFIC_CHANGE_INTERVAL, j, a, method.traffic.lambda_out, Parameters.h) / (numFogNodes * numServices);
                }
            }
        }

        for (int a = 0; a < numServices; a++) {
            for (int j = 0; j < numFogNodes; j++) {
                // Cost of container deployment and cost of violation
                if (method.x_backup[a][j] == 0 && method.x[a][j] == 1) {
                    costF[5] += Cost.costDep(j, a, Parameters.L_S) / (numFogNodes * numServices);
                    costF[6] += Cost.costViol(Parameters.TRAFFIC_CHANGE_INTERVAL, a, j, method.Vper[a], Parameters.q, method.traffic.lambda_in) / (numFogNodes * numServices);
                }

                // Cost of services which could be deployed on a fog node, but didn't
                if(!method.deployMakesSense(a, j) && method.x[a][j] == 1)
                    costF[7] += 1;

                // Cost of services which could be released from a fog node, but didn't
                if(!method.releaseMakesSense(a, j) && method.x[a][j] == 0)
                    costF[8] += 1;
            }
        }

        // Cost of fog nodes which still have free resources
        // This cost hopefully helps to aggregate services on less fog nodes and save more energy
        for (int j = 0; j < numFogNodes; j++)
            if (method.fogHasFreeResources(j))
                costF[9] += 1;

        this.cost = costF;
    }

    /**
     * This method initializes best personal cost with current cost
     */
    void initCbest() {
        this.cbest = this.cost.clone();
    }

    /**
     * This method initializes best personal position with current position
     */
    void initPbest() {
        this.pbest = this.position.clone();
    }

    /**
     * This method update velocity values
     *
     * @param gbest Best Global Particle
     */
    void updateVelocity(MOParticle gbest, int iteration, int max_iteration) {
        Random r = new Random();
        double w = 0.9 - (iteration*1. / max_iteration) * (0.9 - 0.4);
        double r1 = 2.5 - (iteration*1. / max_iteration) * (2.5 - 0.5);
        double r2 = 0.5 - (iteration*1. / max_iteration) * (0.9 - 0.4);
        for(int i = 0; i<this.numberOfVariables; i++)
            this.velocity.values[i] = w * this.velocity.values[i] + r1 * r.nextDouble() * (this.pbest.values[i] - this.position.values[i]) + r2 * r.nextDouble() * (gbest.getPosition()[i] - this.position.values[i]);
    }

    /**
     * This method forces velocity values to be in range (vmin, vmax)
     */
    void restrictVelocity(){
        for(int i = 0; i<this.numberOfVariables; i++){
            if(this.velocity.values[i] > this.vmax)
                this.velocity.values[i] = this.vmax;
            else if(this.velocity.values[i] < this.vmin)
                this.velocity.values[i] = this.vmin;
        }
    }

    /**
     * This method update position values w.r.t applying sigmoid function
     * to velocity values to make it ready for binary position space
     */
    void updatePosition() {
        Random r = new Random();
        double randNum = r.nextDouble();

        for(int a=0; a<this.numServices; a++) {
            for(int i=0; i<numFogNodes; i++) {
                boolean canDEPLOY = sigmoid(this.velocity.values[a*numFogNodes + i]) > randNum;

                if (canDEPLOY && method.x[a][i]==0 && method.fogHasFreeResources(i) && method.Vper[a] > 1 - Parameters.q[a] && method.deployMakesSense(a, i)) {
                    this.position.values[a*numFogNodes + i] = 1;
                    method.x[a][i] = 1;

                    // This function should be called every time the placement variables for a service change
                    method.placementUpdatedForService(a);
                    // Calculate delay Violation Percentage. (Percentage of IoT requests that don't meet the delay requirement for service i (V^%_a))ˇ
                    Violation.calcViolation(a, method);
                }
            }

            for(int i=0; i<numFogNodes; i++) {
                boolean canRELEASE = sigmoid(this.velocity.values[a*numFogNodes + i]) <= randNum;

                if (canRELEASE && method.x[a][i] == 1 && method.Vper[a] <= 1 - Parameters.q[a] && method.releaseMakesSense(a, i)) {
                    this.position.values[a*numFogNodes + i] = 0;
                    method.releaseServiceSafelyFromFogNodes(a, i);

                    // This function should be called every time the placement variables for a service change
                    method.placementUpdatedForService(a);
                    // Calculate delay Violation Percentage. (Percentage of IoT requests that don't meet the delay requirement for service i (V^%_a))ˇ
                    Violation.calcViolation(a, method);
                }
            }
        }

        for(int i=0; i<this.numServices; i++)
            method.deployOrReleaseCloudService(i);

//        for(int i=0; i<this.numberOfVariables; i++){
//            int service_a = i / (numFogNodes + numCloudServers);
//            int part = i % (numFogNodes + numCloudServers);
//            boolean canDEPLOY = sigmoid(this.velocity.values[i]) > r.nextDouble();
//
//            if(part < numFogNodes) { // Should work on fog nodes
//                int node_j = part;
//                if (canDEPLOY && method.x[service_a][node_j] == 0 && method.fogHasFreeResources(node_j) && method.deployMakesSense(service_a, node_j)) {
//                    this.position.values[i] = 1;
//                    method.x[service_a][node_j] = 1;
//                }
//            }
//            else { // Should work on cloud servers
//                // If incoming traffic rate to a cloud server for a particular service is 0, the service could be released to save space.
//                // On the other hand, even if there is small traffic incoming to a cloud server for a particular service, the service must not be removed from the cloud server
//                int server_k = part - numFogNodes;
//                if (canDEPLOY && method.traffic.lambdap_in[service_a][server_k] > 0) {
//                    this.position.values[i] = 1;
//                    method.xp[service_a][server_k] = 1;
//                }
//            }
//
//            // This function should be called every time the placement variables for a service change
//            method.placementUpdatedForService(service_a);
//            // Calculate delay Violation Percentage. (Percentage of IoT requests that don't meet the delay requirement for service i (V^%_a))ˇ
//            Violation.calcViolation(service_a, method);
//        }
//
//        for(int i=0; i<this.numberOfVariables; i++){
//            int service_a = i / (numFogNodes + numCloudServers);
//            int part = i % (numFogNodes + numCloudServers);
//            boolean canDEPLOY = sigmoid(this.velocity.values[i]) > r.nextDouble();
//
//            if(part < numFogNodes) { // Should work on fog nodes
//                int node_j = part;
//                if(!canDEPLOY){
//                    if(method.x[service_a][node_j] == 1 && method.releaseMakesSense(service_a, node_j)) {
//                        this.position.values[i] = 0;
//                        method.releaseServiceSafelyFromFogNodes(service_a, node_j);
//                    }
//                }
//            }
//            else { // Should work on cloud servers
//                // If incoming traffic rate to a cloud server for a particular service is 0, the service could be released to save space.
//                // On the other hand, even if there is small traffic incoming to a cloud server for a particular service, the service must not be removed from the cloud server
//                int server_k = part - numFogNodes;
//                if(method.traffic.lambdap_in[service_a][server_k] == 0){
//                    this.position.values[i] = 0;
//                    method.xp[service_a][server_k] = 0;
//                }
//            }
//
//            // This function should be called every time the placement variables for a service change
//            method.placementUpdatedForService(service_a);
//            // Calculate delay Violation Percentage. (Percentage of IoT requests that don't meet the delay requirement for service i (V^%_a))ˇ
//            Violation.calcViolation(service_a, method);
//        }
    }

    /**
     * This method calculates sigmoid function on a value
     *
     * @param value A value to calculate sigmoid on
     *
     * @return 1 / (1 + e ^ -value)
     */
    private double sigmoid(double value) {
        return 1. / (1 + Math.exp(-value));
    }

    /**
     * This method forces position values to be in range (pos_lower_bound, pos_upper_bound)
     */
    void enforceBounds() {
        Random r = new Random();
        for(int i=0; i<this.numberOfVariables; i++){
            if(this.position.values[i] > this.pos_upper_bound){
                this.position.values[i] = this.pos_lower_bound + (this.pos_upper_bound - this.pos_lower_bound) * r.nextDouble();
                this.velocity.values[i] = (this.pos_upper_bound - this.pos_lower_bound) * r.nextDouble() * 0.5;
            }

            if(this.position.values[i] < this.pos_lower_bound){
                this.position.values[i] = this.pos_upper_bound - (this.pos_upper_bound - this.pos_lower_bound) * r.nextDouble();
                this.velocity.values[i] = (this.pos_upper_bound - this.pos_lower_bound) * r.nextDouble() * 0.5;
            }
        }
    }

    /**
     * This method determines dominated or non-dominated between two vectors of same size
     *
     * @param cost First vector to determine domination
     * @param cost1 Second vector to determine domination
     *
     * @return True, if first vector dominates the second one, False, otherwise
     */
    static boolean Dominates(double[] cost, double[] cost1) {
        if(check_less_than_equal(cost, cost1))
            return check_any_less_than(cost, cost1);

        return false;
    }

    /**
     * This method check to see if there exists an element of first vector less than any element of second vector
     *
     * @param cost First vector to check
     * @param cost1 Second vector to check
     *
     * @return True, if there exists any element, False, otherwise
     */
    private static boolean check_any_less_than(double[] cost, double[] cost1) {
        for(int i=0; i<cost.length; i++)
            if(cost[i] < cost1[i])
                return true;

        return false;
    }

    /**
     * This method check to see if there exists an element of first vector less than or equal to any element of second vector
     *
     * @param cost First vector to check
     * @param cost1 Second vector to check
     *
     * @return True, if there exists any element, False, otherwise
     */
    private static boolean check_less_than_equal(double[] cost, double[] cost1) {
        for(int i=0; i<cost.length; i++)
            if(cost[i] > cost1[i])
                return false;

        return true;
    }

    /**
     * This method determines dominated or non-dominated between two vectors of same size
     *
     * @param cost First vector to determine domination
     * @param cbest Second vector to determine domination, in this case, best personal cost
     *
     * @return True, if first vector dominates the second one, False, otherwise
     */
    private boolean Dominates2(double[] cost, double[] cbest) {
        if(check_bigger_than_equal(cost, cbest))
            return check_any_bigger_than(cost, cbest);

        return false;
    }

    /**
     * This method check to see if there exists an element of first vector bigger than any element of second vector
     *
     * @param cost First vector to check
     * @param cbest Second vector to check, in this case, best personal cost
     *
     * @return True, if there exists any element, False, otherwise
     */
    private boolean check_any_bigger_than(double[] cost, double[] cbest) {
        for(int i=0; i<cost.length; i++)
            if(cost[i] > cbest[i])
                return true;

        return false;
    }

    /**
     * This method check to see if there exists an element of first vector bigger than or equal to any element of second vector
     *
     * @param cost First vector to check
     * @param cbest Second vector to check, in this case, best personal cost
     *
     * @return True, if there exists any element, False, otherwise
     */
    private boolean check_bigger_than_equal(double[] cost, double[] cbest) {
        for(int i=0; i<cost.length; i++)
            if(cost[i] < cbest[i])
                return false;

        return true;
    }

    /**
     * This method uses Pareto Dominance to update best personal cost and position
     */
    void ParetoDominance() {
        Random r = new Random();
        if(Dominates(this.cost, this.cbest)){ // Determine domination between cost and best personal cost
            this.pbest = this.position.clone();
            this.cbest = this.cost.clone();
        }
        else if(Dominates2(this.cost, this.cbest)){
            // Do nothing
        }
        else if(r.nextDouble() >= 0.5){ // If none of them happens this strategy is used
            this.pbest = this.position.clone();
            this.cbest = this.cost.clone();
        }
    }

    /**
     * Get personal best values for position
     *
     * @return Personal Best Position
     */
    private double[] getPbest(){
        return pbest.values.clone();
    }

    /**
     * Get a copy of particle and return it
     *
     * @return Copy/Clone of this particle
     */
    @Override
    public MOParticle clone(){
        MOParticle new_p = new MOParticle(method, this.numberOfObjectives, this.pos_lower_bound, this.pos_upper_bound, this.vmin, this.vmax, this.numberOfVariables, numServices, numFogNodes, numCloudServers);
        new_p.cost = this.cost.clone();
        new_p.cbest = this.cbest.clone();
        new_p.position = this.position.clone();
        new_p.velocity = this.velocity.clone();
        try{
            new_p.pbest = this.pbest.clone();
        } catch (NullPointerException e){
            new_p.pbest = null;
        }

        return new_p;
    }

    /**
     * This method returns a clone of cost vector
     *
     * @return Copy/Clone of cost vector
     */
    double[] getCost() {
        return this.cost.clone();
    }

    /**
     * This method returns a clone of position values/vector
     *
     * @return Copy/Clone of position vector
     */
    private double[] getPosition() {
        return this.position.clone().values;
    }

    /**
     * This method check to see if two particles are in the same position or not
     *
     * @return True, if they're in the same position, False, otherwise
     */
    boolean equals(MOParticle obj) {
        for(int i=0; i<this.numberOfVariables; i++){
            if(getPosition()[i] != obj.getPosition()[i]){
                return false;
            }
        }

        return true;
    }
}
