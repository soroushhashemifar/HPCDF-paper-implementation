
package Scheme;

/**
 *
 * @author Ashkan Y.
 * 
 * This class contains some variables of each method
 */
public class Variable {

    public Integer[][] x; // x_aj
    public int[][] x_backup; // backup of x_aj
    public Integer[][] xp; // x'_ak
    public int[][] v; // v_aj

    public double d[][]; // stores d_aj

    public double Vper[]; // V^%_a

    int numServices, numFogNodes, numCloudServers;

    /**
     * The constructor of the class. Initializes the arrays 
     * @param numServices the number of services
     * @param numFogNodes the number of fog nodes
     * @param numCloudServers the number of cloud servers
     */
    public Variable(int numServices, int numFogNodes, int numCloudServers) {
        x = new Integer[numServices][numFogNodes];
        xp = new Integer[numServices][numCloudServers];
        x_backup = new int[numServices][numFogNodes];
        v = new int[numServices][numFogNodes];
        d = new double[numServices][numFogNodes];
        Vper = new double[numServices];

        this.numServices = numServices;
        this.numFogNodes = numFogNodes;
        this.numCloudServers = numCloudServers;
    }

    public Variable clone(){
        Variable new_variable = new Variable(numServices, numFogNodes, numCloudServers);

        new_variable.x = this.x.clone();
        new_variable.x_backup = this.x_backup.clone();
        new_variable.xp = this.xp.clone();
        new_variable.v = this.v.clone();
        new_variable.d = this.d.clone();
        new_variable.Vper = this.Vper.clone();

        return new_variable;
    }
}

