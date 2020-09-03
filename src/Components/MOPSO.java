package Components;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

/**
 *
 * @author Soroush Hashemi Far
 *
 * This is the class of Multi-Objective Particle Swarm Optimization.
 */
public class MOPSO {

    private int numberOfObjectives, n_div, numberOfParticles, iterations, numberOfVariables, n_rep;
    private double pos_lower_bound, pos_upper_bound, vmax, vmin;
    private LinkedList<MOParticle> particles;
    private LinkedList<MOParticle> archive;
    private MOParticle gbest;
    private int numServices, numFogNodes, numCloudServers;
    private Method method;

    /**
     * Initializes the MO-PSO method
     *
     * @param method Method object, the main class of the simulation, created for PSO
     * @param numberOfObjectives Number of objectives for PSO
     * @param n_div Number of divisions/hypercubes
     * @param numberOfParticles Number of particles used to search
     * @param iterations Epochs to run PSO
     * @param numberOfVariables Number of dimensions of the search space
     * @param pos_lower_bound Minimum value of each element of position vector
     * @param pos_upper_bound Maximum value of each element of position vector
     * @param vmax Maximum value of each element of velocity vector
     * @param vmin Minimum value of each element of velocity vector
     * @param n_rep Number of particles to keep in REPOSITORY/ARCHIVE
     * @param numServices Number of services used in simulation
     * @param numFogNodes Number of fog nodes used in simulation
     * @param numCloudServers Number of cloud servers used in simulation
     */
    MOPSO(Method method, int numberOfObjectives, int n_div, int numberOfParticles, int iterations, int numberOfVariables, double pos_lower_bound, double pos_upper_bound, double vmax, double vmin, int n_rep, int numServices, int numFogNodes, int numCloudServers) {
        this.numberOfObjectives = numberOfObjectives;
        this.n_div = n_div;
        this.numberOfParticles = numberOfParticles;
        this.iterations = iterations;
        this.numberOfVariables = numberOfVariables;
        this.pos_lower_bound = pos_lower_bound;
        this.pos_upper_bound = pos_upper_bound;
        this.vmax = vmax;
        this.vmin = vmin;
        this.n_rep = n_rep;
        this.method = method;
        this.numServices = numServices;
        this.numFogNodes = numFogNodes;
        this.numCloudServers = numCloudServers;
    }

    /**
     * Run the MO-PSO method
     *
     * @return Global best (gbest) particle
     */
    public MOParticle run(){
        // Parts 1 and 2 of original paper
        this.particles = new LinkedList<>();
        initialize();

        // Part 3 and 6 of original paper
        for(MOParticle p: this.particles) {
            p.initPbest(); // Initialize personal best position
            p.updatePosition(); // To redefine method variables before using in evaluation function
            p.evaluate_cost_function();
            p.initCbest(); // Initialize personal best cost values
        }

        // Part 4 of original paper
        this.archive = determineDomination(this.particles);

        // Part 5 of original paper
        // Generate hypercubes, index and subindex matrices
        double[][] grid = generateHypercubes();
        methodsReturnObjects mro = gridIndex(this.archive, grid);
        int[] index = mro.oneDarray;
        int[][] subindex = mro.twoDarray;

        for(int iter=0; iter<this.iterations; iter++){
            if(iter % (this.iterations / 10) == 0)
                System.out.println("Iteration: " + iter);

            // Parts 7.a to 7.c of original paper
            for(MOParticle p: this.particles){
                int h = selectGlobalBest(index);
                gbest = this.archive.get(h);
                p.updateVelocity(gbest, iter, this.iterations);
                p.restrictVelocity();
                p.updatePosition();
                p.enforceBounds();
            }

            // Part 7.d of original paper
            for(MOParticle p: this.particles){
                p.evaluate_cost_function();
                p.ParetoDominance();
            }

            // Part 7.e of original paper
            LinkedList<MOParticle> ndpop = determineDomination(this.particles);
            mro = gridIndex(ndpop, grid);
            int[] new_index = mro.oneDarray;
            int[][] new_subindex = mro.twoDarray;

            mro = archiveController(ndpop, index, subindex, new_index, new_subindex);
            index = mro.oneDarray;
            subindex = mro.twoDarray;

            boolean flag = false;
            int bigger_than_grid_counter = 0;
            int less_than_grid_counter  = 0;
            for (MOParticle particle: this.archive) {
                for (int i = 0; i < this.numberOfObjectives; i++) {
                    if (grid[i][1] < particle.getCost()[i])
                        bigger_than_grid_counter += 1;

                    if (particle.getCost()[i] < grid[i][grid[i].length - 2])
                        less_than_grid_counter += 1;

                    if(bigger_than_grid_counter >= 1 && less_than_grid_counter >= 1){
                        grid = generateHypercubes();
                        mro = gridIndex(this.archive, grid);
                        index = mro.oneDarray;
                        subindex = mro.twoDarray;
                        flag = true;
                        break;
                    }
                }

                if(flag)
                    break;
            }

            if(this.archive.size() > this.n_rep) {
                mro = removeParticles(index, subindex);
                index = mro.oneDarray;
                subindex = mro.twoDarray;
            }
        }

        int h = selectGlobalBest(index);

        return this.archive.get(h);
    }

    /**
     * Initializes particles of original population
     */
    private void initialize(){
        for(int i=0; i<this.numberOfParticles; i++)
            this.particles.add(new MOParticle(this.method, this.numberOfObjectives, this.pos_lower_bound, this.pos_upper_bound, this.vmin, this.vmax, this.numberOfVariables, numServices, numFogNodes, numCloudServers));
    }

    private LinkedList<MOParticle> determineDomination(LinkedList<MOParticle> particles) {
        return determineDomination(particles, false).particles;
    }

    /**
     * Determine non-dominated particles in given population
     *
     * @param particles The population to find non-dominated particles in
     * @param return_index if true, return index together with new population
     *
     * @return New index and population of non-dominated particles encapsulated in a methodsReturnObjects object
     */
    private methodsReturnObjects determineDomination(LinkedList<MOParticle> particles, boolean return_index) {
        LinkedList<MOParticle> ndpop = new LinkedList<>();

        // Create array of non-dominated particles, initialized with all zeros
        // 0 = non-dominated, 1 = dominated by others
        int[] pop_IsDominated = new int[particles.size()];
        for(int i=0; i<pop_IsDominated.length; i++)
            pop_IsDominated[i] = 0;

        // Determine domination of particles using function MOParticle.Dominates
        for(int i=0; i<particles.size(); i++){
            for(int j=0; j<particles.size(); j++){
                if(i != j){
                    if(MOParticle.Dominates(particles.get(j).getCost(), particles.get(i).getCost())){
                        pop_IsDominated[i] = 1;
                        break;
                    }
                }
            }
        }

        // Create an array of non-dominated particles only and an array for their indices in original population
        LinkedList<Integer> index = new LinkedList<>();
        for(int i=0; i<particles.size(); i++){
            if(pop_IsDominated[i] == 0){
                ndpop.add(particles.get(i).clone());
                index.add(i);
            }
        }
        int[] new_index = index.stream().mapToInt(Integer::intValue).toArray(); // Convert index from type LinkedList<Integer> to int[]

//        System.out.println("domination vector " + Arrays.toString(pop_IsDominated));
//        System.out.println("before " + ndpop.size());

        // Remove duplicated particles from ndpop array of non-dominated particles
        int it = 0;
        while (it < ndpop.size()){
            int uIdx = 0;
            for(MOParticle p: ndpop){
                if(p.equals(ndpop.get(it))) {
                    uIdx += 1;

                    if (uIdx > 1) {
                        ndpop.remove(it);
                        int[] temp_new_index = new int[new_index.length-1];
                        System.arraycopy(new_index, 0, temp_new_index, 0, it);
                        System.arraycopy(new_index, it+1, temp_new_index, it, new_index.length-it-1);
                        new_index = temp_new_index.clone();
                        it = 0;
                        break;
                    }
                }
            }

            it += 1;
        }

//        System.out.println("after " + ndpop.size());

        return new methodsReturnObjects(new_index, ndpop);
    }

    /**
     * Generate Hypercubes according to original paper
     *
     * @return Grid of generated hypercubes
     */
    private double[][] generateHypercubes() {
        double[][] grid = new double[this.numberOfObjectives][this.n_div];

        for(int i=0; i<this.numberOfObjectives; i++){
            // Find indices of particles with minimum and maximum cost value
            int min_index=0, max_index=0;
            for(int j=0; j<this.archive.size(); j++) {
                if (this.archive.get(j).getCost()[i] < this.archive.get(min_index).getCost()[i])
                    min_index = j;
                else if (this.archive.get(j).getCost()[i] > this.archive.get(max_index).getCost()[i])
                    max_index = j;
            }

            // Fill first column of grid with -infinity
            grid[i][0] = -1 * Double.MAX_VALUE;

            // Set middle columns of grid equal to "n_div-2" numbers between minimum and maximum cost particles in archive
            double[] temp = linspace(this.archive.get(min_index).getCost()[i], this.archive.get(max_index).getCost()[i], n_div-2).clone();
            System.arraycopy(temp, 0, grid[i], 1, temp.length);

            // Fill last column of grid with +infinity
            grid[i][this.n_div-1] = Double.MAX_VALUE;
        }

        return grid;
    }

    /**
     * Generate numbers in a range with same distance between them
     *
     * @param min Minimum value of the range
     * @param max Maximum value of the range
     * @param points Number of points to generate in this range
     *
     * @return Array of numbers
     */
    private static double[] linspace(double min, double max, int points) {
        double[] d = new double[points];

        for (int i = 0; i < points; i++)
            d[i] = min + i * (max - min) / (points - 1);

        return d;
    }

    /**
     * Generate index and subindex arrays
     *
     * @param pop A population of particles to find index and subindex arrays from
     * @param grid A matrix contain Hypercubes (according to MOPSO paper)
     *
     * @return index and subindex arrays encapsulated in a methodsReturnObjects object
     */
    private methodsReturnObjects gridIndex(LinkedList<MOParticle> pop, double[][] grid) {
        // Make and fill subindex matrix
        int[][] subindex = new int[pop.size()][this.numberOfObjectives]; // Keeps minimum index of each particle's cost element with corresponding value in grid's rows
        for(int i=0; i<pop.size(); i++){
            int[] subIdx = new int[this.numberOfObjectives]; // Keeps indices of grid element more than corresponding particle's cost for each row of grid
            for(int j=0; j<this.numberOfObjectives; j++) {
                LinkedList<Integer> temp = new LinkedList<>();
                for(int k=0; k<this.n_div; k++) { // Compare each element of grid row j with j-th element of particle's cost
                    if (pop.get(i).getCost()[j] <= grid[j][k])
                        temp.add(k);
                }

                subIdx[j] = Collections.min(temp);
            }

            // Append subIdx array to end of subindex matrix
            System.arraycopy(subIdx, 0, subindex[i], 0, this.numberOfObjectives);
        }

        // Make and fill index array
        int[] index = new int[pop.size()]; // Index of each subindex's element in flattened form of a matrix with dimensions equal to number of objectives
        // Find index of element in ravel/flattened form of matrix
        // This part is equal to ravel_multi_index function from Numpy Library in Python programming language
        for(int i=0; i<pop.size(); i++){
            int sum = 0;
            sum += subindex[i][this.numberOfObjectives-1];

            for(int j=2; j<this.numberOfObjectives+1; j++){
                int prod = 1;
                prod *= subindex[i][this.numberOfObjectives-j];
                for(int k=0; k<j-1; k++)
                    prod *= this.n_div;

                sum += prod;
            }

            index[i] = sum;
        }

        return new methodsReturnObjects(index, subindex);
    }

    private int selectGlobalBest(int[] index) {
        return selectGlobalBest(index, 0);
    }

    /**
     * Select best or worst particle in global archive
     *
     * @param index Index of each subindex's element in flattened form of a matrix with dimensions equal to number of objectives
     * @param type Select best, if type=0, and select worst, if type=1
     *
     * @return Index of specific particle
     */
    private int selectGlobalBest(int[] index, int type) {
        double[] fitness = new double[this.archive.size()];

        // Convert index array from type int[] to Integer[] to be used in Collections.frequency function
        Integer[] Index = IntStream.of(index).boxed().toArray(Integer[]::new);

//        System.out.println(this.archive.size() + ", " + index.length);
        // Determine weight/fitness for each element in index array
        int x = 10;
        for(int i=0; i<index.length; i++){
            int nIdx = Collections.frequency(Arrays.asList(Index), index[i]);
            // Give more chance to more repeated indices to find BEST particles, give less chance to more repeated indices to find WORST particles
            fitness[i] = type == 0 ? x*1. / nIdx : x * nIdx;
        }

        // Calculate cumulative sum on fitness array
        Double[] arr = DoubleStream.of(fitness).boxed().toArray(Double[]::new);
        Arrays.parallelPrefix(arr, Double::sum);
        List<Double> rouletteWheelList = Arrays.asList(arr);
        double[] rouletteWheel = rouletteWheelList.stream().mapToDouble(d -> d).toArray();

        // Return index of first element with more value than a random value, if it exists
        Arrays.sort(rouletteWheel);
        Random r = new Random();
        double temp = (rouletteWheel[rouletteWheel.length-1] - rouletteWheel[0]) * r.nextDouble();
        for(int i=0; i<rouletteWheel.length; i++){
            if(temp <= rouletteWheel[i])
                return i;
        }

        // If that specific value doesn't exist, return last element
        return r.nextInt(rouletteWheel.length);
    }

    /**
     * Remove worst particles from archive/repository
     *
     * @param ndpop Non-dominated particles in population
     * @param index Index of each subindex's element in flattened form of a matrix with dimensions equal to number of objectives
     * @param subindex Minimum of each cost element with corresponding value in grid's rows for each particle in archive
     * @param new_index Index of each subindex's element in flattened form of a matrix with dimensions equal to number of objectives
     * @param new_subindex Minimum of each cost element with corresponding value in grid's rows for each particle in archive
     *
     * @return New index and new subindex arrays encapsulated in a methodsReturnObjects object
     */
    private methodsReturnObjects archiveController(LinkedList<MOParticle> ndpop, int[] index, int[][] subindex, int[] new_index, int[][] new_subindex) {
        // Append ndpop to global archive
        this.archive.addAll(ndpop);

        // Append new_index array to index array
        int[] new__index = Arrays.copyOf(index, index.length + new_index.length);
        System.arraycopy(new_index, 0, new__index, index.length, new_index.length);

        // Append new_subindex matrix to subindex matrix
        int[][] new__subindex = new int[subindex.length + new_subindex.length][this.numberOfObjectives];
        System.arraycopy(subindex, 0, new__subindex, 0, subindex.length);
        System.arraycopy(new_subindex, 0, new__subindex, subindex.length, new_subindex.length);

        // Find non-dominated particles and store their indices in temp_index array
        methodsReturnObjects mro = determineDomination(this.archive, true);
        LinkedList<MOParticle> new_archive = mro.particles;
        int[] temp_index = mro.oneDarray;

        // Update index array
        int[] indexRep = Arrays.stream(temp_index).map(e -> new__index[e]).toArray();

        // Update subindex array
        int[][] subindicesRep = new int[temp_index.length][this.numberOfObjectives];
        for(int i=0; i<temp_index.length; i++)
            System.arraycopy(new__subindex[temp_index[i]], 0, subindicesRep[i], 0, this.numberOfObjectives);

        // Remove repeated particles from archive
        int it = 0;
        while (it < new_archive.size()){
            int uIdx = 0;
            for(MOParticle p: new_archive){
                if(p.equals(new_archive.get(it))) {
                    uIdx += 1;

                    if (uIdx > 1) {
                        new_archive.remove(it);
                        it = 0;
                        break;
                    }

                }
            }

            it += 1;
        }


        this.archive = (LinkedList<MOParticle>) new_archive.clone();

        return new methodsReturnObjects(indexRep, subindicesRep);
    }

    /**
     * Remove worst particles from archive/repository
     *
     * @param index Index of each subindex's element in flattened form of a matrix with dimensions equal to number of objectives
     * @param subindex Minimum of each cost element with corresponding value in grid's rows for each particle in archive
     *
     * @return index and subindex arrays encapsulated in a methodsReturnObjects object
     */
    private methodsReturnObjects removeParticles(int[] index, int[][] subindex) {
        int h;

        for(int i=0; i<this.n_rep; i++){
            h = selectGlobalBest(index, 1); // find worst particle in global archive

            this.archive.remove(h);

            // Remove index h from index array
            List<Integer> indexList = IntStream.of(index).boxed().collect(Collectors.toList());
            indexList.remove(h);
            index = indexList.stream().mapToInt(Integer::intValue).toArray();

            // Remove row index h from subindex matrix
            int[][] new_subindex = new int[subindex.length-1][this.numberOfObjectives];
            System.arraycopy(subindex, 0, new_subindex, 0, h);
            System.arraycopy(subindex, h+1, new_subindex, h, subindex.length-h-1);
            subindex = new_subindex.clone();
//            System.out.println("inside remove, i=" + i);
//            System.out.println("index size=" + index.length);
        }

        return new methodsReturnObjects(index, subindex);
    }
}

/**
 *
 * @author Soroush Hashemi Far
 *
 * This class is used to carry different data structures between methods of MOPSO class
 */
class methodsReturnObjects{

    int[] oneDarray;
    int[][] twoDarray;
    LinkedList<MOParticle> particles;

    methodsReturnObjects(int[] oneDarray, int[][] twoDarray) {
        this.oneDarray = oneDarray.clone();
        this.twoDarray = twoDarray.clone();
    }

    methodsReturnObjects(int[] oneDarray, LinkedList<MOParticle> particles) {
        this.oneDarray = oneDarray.clone();
        this.particles = (LinkedList<MOParticle>) particles.clone();
    }
}