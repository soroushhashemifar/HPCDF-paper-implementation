package DTMC;

import Scheme.Parameters;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

/**
 *
 * @author Ashkan Y.
 *
 * This class has methods that constructs a DTMC based on a traffic pattern that
 * is read from input (e.g. a file)
 */
public class DTMCconstructor {

    private Scanner in;
    private final int NUM_OF_STATES = 30; // number of states of DTMC
    private final String FILE_ADDRESS = "/home/soroush/Desktop/FogPlan/FogPlan-simulator-master/Traffic-Trace/traffic-pattern.txt";
    private static final double SMOOTHING_NUMBER = 0.000000000001d; // used so that we will not have absolute 0 as traffic rate

    private double trafficRateIncrementSize; // the size of the steps between two contiguus traffic rates

    private double averageTrafficRate; // average traffic rate after creating a trace based on DTMC

    public DTMC dtmc;

    private double minRate, maxRate; // min and max of traffic rate

    /**
     * The constructor of the Discrete Time Markov Chain (DTMC)
     */
    public DTMCconstructor() {
        dtmc = new DTMC(NUM_OF_STATES);
        int[] trafficLevel = null;
        try {
            trafficLevel = readTrafficFromFile(FILE_ADDRESS, NUM_OF_STATES);
        } catch (FileNotFoundException ex) {
            System.out.println("The traffic pattern file is not found here: " + FILE_ADDRESS);
            ex.printStackTrace();
        }

        double[] TrafficRateInState = setTrafficRateInState(minRate, NUM_OF_STATES, trafficRateIncrementSize);
        dtmc.setTrafficRateInState(TrafficRateInState);
        // rate[][] will have number of times traffic changes from traffic level i to j
        int[][] rates = new int[NUM_OF_STATES][NUM_OF_STATES];
        for (int i = 0; i < NUM_OF_STATES; i++) {
            for (int j = 0; j < NUM_OF_STATES; j++) {
                rates[i][j] = 0;
            }
        }
        for (int i = 0; i < trafficLevel.length - 1; i++) {
            rates[trafficLevel[i]][trafficLevel[i + 1]]++;
        }
        dtmc.setTransitionRates(rates);
        // 'Next' arrays
        dtmc.configArrays();
    }

    /**
     * Constructs a DTMC based on a traffic pattern that is read from input
     * (e.g. a file). If the FileAddress is empty (""), the input is read from
     * console
     *
     * @param FileAddress the location of the traffic trace file
     * @param numberOfStates the number of states in the DTMC
     * @throws java.io.FileNotFoundException if the traffic trace file is not
     * found
     * @return an array indicating the traffic rate in each level (state)
     */
    private int[] readTrafficFromFile(String FileAddress, int numberOfStates) throws FileNotFoundException {
        if (FileAddress.equalsIgnoreCase("")) {
            in = new Scanner(System.in);
        } else {
            File inputFile = new File(FileAddress);
            in = new Scanner(inputFile);
        }
        ArrayList<Double> trafficValue = new ArrayList<>(); // this will store the actual traffic values
        double input;
        averageTrafficRate = in.nextDouble(); // the first number in the traffic file is the average traffic rate (but it is not normalized yet)
        while (in.hasNext()) {
            input = in.nextDouble();
            trafficValue.add(input);
        }
        return extractLevelNumberFromTrace(trafficValue, numberOfStates);
    }

    /**
     * This function extracts the amount of traffic in each state (level of
     * traffic) from the traffic trace
     *
     * @param trafficValue the time stamped traffic trace values
     * @param numberOfStates the number of states in DTMC
     * @return an array indicating the traffic rate in each level (state)
     */
    private int[] extractLevelNumberFromTrace(ArrayList<Double> trafficValue, int numberOfStates) {
        normalizeTraceTraffic(trafficValue);
        findMinAndMax(trafficValue); // this updates the min and max to the new normalized numbers (note that the original min and max might be somehting like 1223 and 10000, but we want them from here to be NORMALIZED min and max)

        int[] trafficLevel = new int[trafficValue.size()];
        trafficRateIncrementSize = (maxRate - minRate) / (numberOfStates - 1); // number of traffic rates = number of horizontal lines on traffic graph + 1
        for (int i = 0; i < trafficValue.size(); i++) {
            trafficLevel[i] = (int) Math.floor((trafficValue.get(i) - minRate + SMOOTHING_NUMBER) / trafficRateIncrementSize);
        }
        return trafficLevel;
    }

    /**
     * This method sets the appropriate traffic value to each state of the DTMC
     *
     * @param minTrafficRate minimum amount of traffic (normalized)
     * @param numberOfStates the number of DTMC's state
     * @param trafficRateIncrementSize step size of traffic rate increments
     * @return return an array containing the appropriate traffic value to each
     * state of the DTMC
     */
    private double[] setTrafficRateInState(double minTrafficRate, int numberOfStates, double trafficRateIncrementSize) {
        double[] TrafficRateInState = new double[numberOfStates];
        for (int i = 0; i < numberOfStates; i++) {
            TrafficRateInState[i] = (minTrafficRate + (i * trafficRateIncrementSize));
        }
        return TrafficRateInState;
    }

    /**
     * This will normalized the traffic values between [0, TRAFFIC_NORM_FACTOR]
     *
     * @param trafficTrace
     */
    private void normalizeTraceTraffic(ArrayList<Double> trafficTrace) {
        findMinAndMax(trafficTrace);
        for (int i = 0; i < trafficTrace.size(); i++) {
            trafficTrace.set(i, (trafficTrace.get(i) - minRate + SMOOTHING_NUMBER) / (maxRate - minRate) * Parameters.TRAFFIC_NORM_FACTOR);
        }
        averageTrafficRate = (averageTrafficRate - minRate + SMOOTHING_NUMBER) / (maxRate - minRate) * Parameters.TRAFFIC_NORM_FACTOR;
    }

    /**
     * Finds the minimum and maximum amount of traffic in the trace
     *
     * @param trace the input trace
     */
    private void findMinAndMax(ArrayList<Double> trace) {
        minRate = Double.MAX_VALUE;
        maxRate = Double.MIN_VALUE;
        for (Double traffic : trace) {
            if (traffic < minRate) {
                minRate = traffic;
            }
            if (traffic > maxRate) {
                maxRate = traffic;
            }
        }
    }
}
