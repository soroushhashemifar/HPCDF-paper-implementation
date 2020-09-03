package Components;

import Scheme.Parameters;
import Utilities.ArrayFiller;
import Utilities.RandomGenerator;
import com.sun.istack.internal.NotNull;
import java.util.*;

class HybridSOPsoCro {
    private int numOfParticles, variables;
    private int epochs;
    private double inertia;
    private double cognitiveComponent;
    private double socialComponent;
    private static final double DEFAULT_INERTIA = 0.729;
    private static final double DEFAULT_COGNITIVE = 1.49445; // Cognitive component.
    private static final double DEFAULT_SOCIAL = 1.49445; // Social component.
    private LinkedList<HybridSOParticle> particles;
    private static HybridSOParticle optimal = null;
    private double KELossRate;
    private double interRate;
    private int init_ke;
    private int gamma;
    private int numServices, numFogNodes, numCloudServers;
    private double vmax, vmin;
    private Method method;
    private double acceleration_coefficient;

    /**
     * Construct the Swarm with default values.
     * @param particles     the number of particles to create
     * @param epochs        the number of generations
     * @param variables     the number of variables
     */
    HybridSOPsoCro(Method method, int particles, int epochs, int variables, int initKE, double KElossRate, double interRate, int gamma, double vmax, double vmin, int numServices, int numFogNodes, int numCloudServers, double acceleration_coefficient) {
        this(method, particles, epochs, variables, DEFAULT_INERTIA, DEFAULT_COGNITIVE, DEFAULT_SOCIAL, initKE, KElossRate, interRate, gamma, vmax, vmin, numServices, numFogNodes, numCloudServers, acceleration_coefficient);
    }

    /**
     * Construct the Swarm with custom values.
     * @param numOfParticles     the number of particles to create
     * @param epochs        the number of generations
     * @param variables     the number of variables
     * @param inertia       the particles resistance to change
     * @param cognitive     the cognitive component or introversion of the particle
     * @param social        the social component or extroversion of the particle
     */
    private HybridSOPsoCro(Method method, int numOfParticles, int epochs, int variables, double inertia, double cognitive, double social, int initKE, double KElossRate, double interRate, int gamma, double vmax, double vmin, int numServices, int numFogNodes, int numCloudServers, double acceleration_coefficient) {
        this.numOfParticles = numOfParticles;
        this.epochs = epochs;
        this.inertia = inertia;
        this.cognitiveComponent = cognitive;
        this.socialComponent = social;
        this.variables = variables;
        this.gamma = gamma;
        this.KELossRate = KElossRate;
        this.interRate = interRate;
        this.init_ke = initKE;
        this.method = method;
        this.numServices = numServices;
        this.numFogNodes = numFogNodes;
        this.numCloudServers = numCloudServers;
        this.vmax = vmax;
        this.vmin = vmin;
        this.acceleration_coefficient = acceleration_coefficient;
    }

    /**
     * Execute the algorithm.
     */
    public HybridSOParticle run() {
        optimal = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);

        particles = initialize();
        for (HybridSOParticle p : particles) {
            p.setKe(init_ke);
            p.updatePersonalBest();
        }

        for (HybridSOParticle p : particles) {
            updateGlobalBest(p);
        }

        Random r = new Random();

        for (int i = 0; i < epochs; i++) {
//            System.out.println(optimal.getBestEval());
            HybridSOParticle particle = particles.get(RandomGenerator.genIntegerRandomBetween(0, particles.size()-1));
            if (particle.getPSOCoe() > gamma) {
                PSOUpdate(i, epochs);
                particle.resetPSOCoe();
//                System.out.println("pso");
            }
            else {
                double random = r.nextDouble();
                if (random > interRate) {
                    HybridSOParticle particle2 = particles.get(RandomGenerator.genIntegerRandomBetween(0, particles.size()-1));
                    while (particle.equals(particle2))
                        particle2 = particles.get(RandomGenerator.genIntegerRandomBetween(0, particles.size()-1));

                    interMoleculeIneffectiveCollision(particle, particle2);
//                    System.out.println("inter");
                }
                else {
                    onWallIneffectiveCollision(particle);
//                    System.out.println("on-wall");
                }
            }

            HybridSOParticle.applySchedule(optimal.method, optimal, numServices, numFogNodes, 1);

            boolean terminate = true;
            for(int a=0; a<numServices; a++){
                double service_delay = 0;
                for(int j=0; j<numFogNodes; j++)
                    service_delay += optimal.method.delay.calcServiceDelay(a, j);
                service_delay = service_delay / numFogNodes;

//                System.out.println(optimal.method.Vper[a] + ", " + (1 - Parameters.q[a]) + ", " + service_delay + ", " + Parameters.th[a]);
                if(optimal.method.Vper[a] > (1 - Parameters.q[a]) || service_delay > Parameters.th[a]) {
//                    System.out.println(optimal.method.Vper[a] + ", " + (1 - Parameters.q[a]) + ", " + service_delay + ", " + Parameters.th[a]);
                    terminate = false;
                    break;
                }
            }

            if(terminate) {
                System.out.println("terminate criteria is satisfied at iteration " + i);
                break;
            }

//            System.out.println(optimal.method.getUtilizationFog(0.5) * 100.);
        }

        HybridSOParticle.applySchedule(method, optimal, numServices, numFogNodes, 1);
//        HybridSOParticle.applySchedule(optimal.method, optimal, numServices, numFogNodes, 1);

        return optimal;
    }

    /**
     * Create a set of particles, each with random starting positions.
     * @return  an array of particles
     */
    private LinkedList<HybridSOParticle> initialize () {
        LinkedList<HybridSOParticle> particles = new LinkedList<>();
        for (int i = 0; i < numOfParticles; i++) {
            HybridSOParticle particle = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);
            particles.add(particle);
        }

        return particles;
    }

    /**
     * Update the global best solution if a the specified particle has
     * a better solution
     * @param particle  the particle to analyze
     */
    private void updateGlobalBest (HybridSOParticle particle) {
        HybridSOParticle.applySchedule(particle.method, particle, numServices, numFogNodes, 1);
        if (particle.getBestEval() <= optimal.getBestEval() && particle.getBestViol() <= optimal.getBestViol() && particle.getBestSrvDelay() <= optimal.getBestSrvDelay()) {
            optimal.copy(particle);
        }
    }

    /**
     * Update the velocity of a particle using the velocity update formula
     * @param particle  the particle to update
     */
    private void updateVelocity(HybridSOParticle particle, int iteration, int max_iteration) {
        Vector oldVelocity = particle.getVelocity();
        Vector pBest = particle.getBestPosition().clone();
        Vector gBest = optimal.getBestPosition().clone();
        Vector pos = particle.getPosition().clone();

        double r1 = RandomGenerator.genUniformRandom();
        double r2 = RandomGenerator.genUniformRandom();
        double r3 = RandomGenerator.genUniformRandom();

        // based on paper: Self-Organizing Hierarchical Particle Swarm Optimizer With Time-Varying Acceleration Coefficients
//        inertia = (1.2 - 0.9) * (iteration*1. / (max_iteration-1)) + 0.9; //RandomGenerator.genUniformRandomBetween(0.5, 1.);
        cognitiveComponent = (0.5 - 0.9) * (iteration*1. / (max_iteration-1)) + 0.9; // 2.5 -> 0.5
        socialComponent = (5.5 - 0.5) * (iteration*1. / (max_iteration-1)) + 0.5; // 0.5 -> 2.5
//        double learningFactorComponent = (1.5 - 1.05) * (iteration*1. / (max_iteration-1)) + 1.05; // 1.05 -> 1.5

        // based of paper: Fast Convergence Strategy for Particle Swarm Optimization using Spread Factor
        double minPe = Double.POSITIVE_INFINITY;
        double maxPe = Double.NEGATIVE_INFINITY;
        for(int i=0; i<numOfParticles; i++) {
            double pe = particles.get(i).getPe();
            if(pe > maxPe)
                maxPe = pe;
            else if(pe < minPe)
                minPe = pe;
        }
        double precision = maxPe - minPe;

        Vector avgPosition = new Vector(variables, 0);
        for(int i=0; i<numOfParticles; i++) {
            avgPosition.add(particles.get(i).getPosition());
        }
        avgPosition.div(numOfParticles);
        double sum_ = 0;
        for(int i=0; i<variables; i++) {
            sum_ += Math.pow(avgPosition.values[i] - optimal.getBestPosition().values[i], 2);
        }
        double deviation = Math.sqrt(sum_);
        double spreadFactor = (precision + deviation) / 2.;
        inertia = Math.exp(-1. * iteration / (spreadFactor * max_iteration));

        // The first product of the formula.
        Vector newVelocity = oldVelocity.clone();
        newVelocity.mul(inertia);

        // The second product of the formula.
        pBest.sub(pos);
        pBest.mul(cognitiveComponent);
        pBest.mul(r1);
        newVelocity.add(pBest);

        // The third product of the formula.
        gBest.sub(pos);
        gBest.mul(socialComponent);
        gBest.mul(r2);
        newVelocity.add(gBest);

        // The fourth product of the formula.
        // based on paper: Fast Convergence Particle Swarm Optimization for Functions Optimization
//        double sum = 0;
//        for(int i=0; i<variables; i++)
//            sum += pos.values[i];
//        Vector Pmd_p = new Vector(variables, sum);
//        Pmd_p.sub(pos);
//        Pmd_p.mul(learningFactorComponent);
//        Pmd_p.mul(r3);
//        newVelocity.add(Pmd_p);

//        // based on paper: https://www.hindawi.com/journals/mpe/2018/9235346/
//        newVelocity.mul(acceleration_coefficient);

        particle.setVelocity(newVelocity);

        particle.checkVelocityBoundaries();
    }

    private void PSOUpdate(int iteration, int max_iteration){
        for (HybridSOParticle p : particles){
            updateVelocity(p, iteration, max_iteration);
            p.updatePosition();
            p.updatePersonalBest();
        }

        for (HybridSOParticle p : particles){
            updateGlobalBest(p);
        }
    }

    private void onWallIneffectiveCollision(@NotNull HybridSOParticle particle){
        particle.incrementNumOfHits();
        HybridSOParticle newP = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);
        newP.copy(particle);
        newP.swapPosition();
        newP.setPe(newP.eval());

        double temp = particle.getPe() + particle.getKe() - newP.getPe();
        if(temp > 0){
            double q = RandomGenerator.genUniformRandomBetween(KELossRate, 1);
            newP.setKe(temp * q);
            newP.updatePersonalBest();

            particles.remove(particle);
            newP.incrementPSOCoe();
            particles.addLast(newP);
            updateGlobalBest(newP);
        }
    }

    private void interMoleculeIneffectiveCollision(@NotNull HybridSOParticle particle1, @NotNull HybridSOParticle particle2){
        particle1.incrementNumOfHits();
        particle2.incrementNumOfHits();

        HybridSOParticle newP1 = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);
        HybridSOParticle newP2 = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);
        newP1.copy(particle1);
        newP2.copy(particle2);
        newP1.swapPosition(particle2.getPosition().clone(), particle1.getPosition().clone());
        newP2.swapPosition(particle1.getPosition().clone(), particle2.getPosition().clone());
        newP1.setPe(newP1.eval());
        newP2.setPe(newP2.eval());

        double temp = particle1.getPe() + particle2.getPe() + particle1.getKe() + particle2.getKe() - newP1.getPe() - newP2.getPe();
        if(temp > 0){
            double q = RandomGenerator.genUniformRandom();
            newP1.setKe(temp * q);
            newP2.setKe(temp * (1-q));
            newP1.updatePersonalBest();
            newP2.updatePersonalBest();

            particles.remove(particle1);
            particles.remove(particle2);
            newP1.incrementPSOCoe();
            newP2.incrementPSOCoe();
            particles.add(newP1);
            particles.add(newP2);
            updateGlobalBest(newP1);
            updateGlobalBest(newP2);
        }
    }

//    private void decomposition(@NotNull HybridSOParticle particle){
//        particle.incrementNumOfHits();
//
//        HybridSOParticle newP1 = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);
//        HybridSOParticle newP2 = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);
//        newP1.copy(particle);
//        newP2.copy(particle);
//        newP1.decPositionUpdate();
//        newP2.decPositionUpdate();
//        newP1.setPe(newP1.eval());
//        newP2.setPe(newP2.eval());
//
//        double temp = particle.getPe() + particle.getKe() - newP1.getPe() - newP2.getPe();
//        if(temp >= 0){
//            double q = RandomGenerator.genUniformRandom();
//            newP1.setKe(temp * q);
//            newP2.setKe(temp * (1-q));
//        } else {
//            newP1.setKe((temp + buffer) * RandomGenerator.genUniformRandom() * RandomGenerator.genUniformRandom());
//            newP2.setKe((temp + buffer - newP1.getKe()) * RandomGenerator.genUniformRandom() * RandomGenerator.genUniformRandom());
//            buffer += temp - newP1.getKe() - newP2.getKe();
//        }
//
//        newP1.update();
//        newP2.update();
//
//        particles.remove(particle);
//        newP1.incrementPSOCoe();
//        newP2.incrementPSOCoe();
//        particles.add(newP1);
//        particles.add(newP2);
//        updateOptimal(newP1);
//        updateOptimal(newP2);
//
//        if(newP1.getPe() < newP1.getBestEval()){
//            newP1.setBestPosition(newP1.getPosition().clone());
//            newP1.setBestEval(newP1.getPe());
//        }
//
//        if(newP2.getPe() < newP2.getBestEval()){
//            newP2.setBestPosition(newP2.getPosition().clone());
//            newP2.setBestEval(newP2.getPe());
//        }
//    }
//
//    private void synthesis(@NotNull HybridSOParticle particle1, @NotNull HybridSOParticle particle2){
//        particle1.incrementNumOfHits();
//        particle2.incrementNumOfHits();
//
//        HybridSOParticle newP = new HybridSOParticle(method.clone(), vmax, vmin, variables, numServices, numFogNodes, numCloudServers);
//        newP.copy(particle1);
//
//        newP.synPositionUpdate(particle2);
//        newP.setPe(newP.eval());
//
//        double temp = particle1.getPe() + particle2.getPe() + particle1.getKe() + particle2.getKe() - newP.getPe();
//        if(temp >= 0){
//            newP.setKe(temp);
//            newP.update();
//
//            particles.remove(particle1);
//            particles.remove(particle2);
//            newP.incrementPSOCoe();
//            particles.add(newP);
//            updateOptimal(newP);
//        }
//
//        if(newP.getPe() < newP.getBestEval()){
//            newP.setBestPosition(newP.getPosition().clone());
//            newP.setBestEval(newP.getPe());
//        }
//    }
}
