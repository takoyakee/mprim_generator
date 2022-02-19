/*
    Example code for motion primitive generation using ACADO toolkit.

    References:
        1. Truck model:
           Improved Optimization of Motion Primitives for Motion Planning in State Lattices
        2. Cost function:    
           Equation (4), Lattice-based Motion Planning for a General 2-trailer
           system
*/

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado_math/acado_math_functions.h>

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

int main( ){

    USING_NAMESPACE_ACADO

    int r = 5; //resolution

    //DISCRETISED INTIIAL HEADINGS: 
    std::vector<double> disc_heading = {0,                              // 0 DEG
                                        atan2(5,10),                    // 26.6 DEG
                                        M_PI/4,                         // 45 DEG
                                        M_PI/2 - atan2(5,10),           // 63.4 DEG
                                        M_PI/2,                         // 90 DEG
                                        -atan2(5,10),                   // -26.6 DEG
                                        -M_PI/4,                        // -45 DEG
                                        -atan2(10,5),                   // -63.4 DEG
                                        -M_PI/2};                       // -90 DEG

    
    
    //DISCRETISED STATE: (TEMP) CONSIDERING 90 TO -90 DEGREE INITIAL HEADING
    std::vector<std::vector<int>> disc_position = { {0,r},              // 90 DEG
                                                    {r,2*r},            // 63.4 DEG
                                                    {r,r},              // 45 DEG
                                                    {2*r,r},            // 26.6 DEG 
                                                    {r,0},              // 0 DEG
                                                    {2*r,-r},           // -26.6 DEG 
                                                    {r,-r},             // -45 DEG
                                                    {r,-2*r},           // -63.4 DEG
                                                    {0,-r} };           // -90 DEG
    std::vector<double> all_headings;
    std::vector<VariablesGrid> results;

    /*std::vector< std::vector<double>> states = {{16,0,0,0} , {14,4,0,22.5} , {8,0,0,0},
                                            {16,2,22.5,0}, {12,6,22.5,22.5}, {14,10,22.5,45} ,
                                            {12,12,45.0,45}, {10,8,45.0,22.5}, {8,10,45.0,67.5},
                                            {10,14,67.5,45}, {6,12,67.5,67.5}, {2,16,67.5,90},
                                            {4,14,90,67.5}, {0,8,90.0,90.0}, {0,16,90.0,90.0} }; */

    /*std::vector< std::vector<double>> states = {{16,0,90,90} , {14,4,90,67.5} , {8,0,90,90},
                                            {16,2,67.5,90}, {12,6,67.5,67.5}, {14,10,67.5,45}, {8,4,67.5,67.5} ,
                                            {12,12,45.0,45}, {10,8,45.0,67.5}, {8,10,45.0,22.5}, {6,6,45,45},
                                            {10,14,22.5,45}, {6,12,22.5,22.5}, {2,16,22.5,0}, {4,8,22.5,22.5},
                                            {4,14,0,22.5}, {0,8,0,0}, {0,16,0,0} };*/
    std::vector< std::vector<double>> states = {{32,0,90,90} , {28,8,90,67.5} , {16,0,90,90},
                                            {32,4,67.5,90}, {24,12,67.5,67.5}, {28,20,67.5,45}, {16,8,67.5,67.5} ,
                                            {24,24,45.0,45}, {20,16,45.0,67.5}, {16,20,45.0,22.5}, {12,12,45,45},
                                            {20,28,22.5,45}, {12,24,22.5,22.5}, {4,32,22.5,0}, {8,16,22.5,22.5},
                                            {8,28,0,22.5}, {0,16,0,0}, {0,32,0,0} };


    //open output file
    std::ofstream myfile;
    myfile.open("csv/raw_prim.txt");

    int iteration = 0;

    // INTRODUCE THE CONSTANTS:
    // -------------------------
    const double L    = 15.0   ; // minimum turning radius (?)
    const double vmax = 15.0  ;
    const double x4max = M_PI/4;
    const double x5max = 1.5  ;
    const double umax = 40.0;
    const int lambda = 1;
    int step =  9;  //chosen to be compatible with the number of intervals in SBPL



    for (auto& endpose : states) {
        int pos_y = endpose[0], pos_x = endpose[1];
        double final_heading = M_PI*endpose[2]/180, initial_heading = M_PI*endpose[3]/180;
        // INTRODUCE THE VARIABLES:
        // -------------------------

        DifferentialState x1, x2  ; // x, y positions
        DifferentialState x3, x4  ; // heading, steering angle
        DifferentialState x5      ; // steering angle rate

        Control           v       ; // longitudinal velocity
        Control           u       ; // steering angle acceleration

        Parameter         T       ; //time horizon



        DifferentialEquation  f(0,T);


        // DEFINE A DIFFERENTIAL EQUATION:
        // -------------------------------

        f << dot(x1) == v*cos(x3)             ;
        f << dot(x2) == v*sin(x3)             ;
        f << dot(x3) == v*tan(x4)/L           ;
        f << dot(x4) == x5                    ;
        f << dot(x5)  == u                    ;
        //f << dot(v) == a;

        // DEFINE AN OPTIMAL CONTROL PROBLEM:
        // ----------------------------------

        OCP ocp( 0, T,step);
        //ocp.minimizeLagrangeTerm(T);
        //current: steering angle, steering angle rate, steerng angle acceleration
        ocp.minimizeLagrangeTerm(x4*x4 + 10*x5*x5 + u*u); //ref paper: yaw rate and yaw acceleration


        // DEFINE OCP CONSTRAINTS:
        // ----------------------------------
        
        ocp.subjectTo( f );

        //start state has to be the same
        ocp.subjectTo( AT_START, x1 ==  0.0 ); // start x position
        ocp.subjectTo( AT_START, x2 ==  0.0 ); // start y position
        ocp.subjectTo( AT_START, x3 ==  initial_heading );
        ocp.subjectTo( AT_START, x4 ==  0.0 ); //consider discretising steer angle
        //ocp.subjectTo( AT_START, x5 ==  0.0 ); 
        //ocp.subjectTo( AT_START, v  ==  3.0 );
        ocp.subjectTo( AT_START, u  ==  0.0 );

        ocp.subjectTo( AT_END  , x1 == pos_x ); // goal x position
        ocp.subjectTo( AT_END  , x2 == pos_y ); // goal y position
        ocp.subjectTo( AT_END  , x3 == final_heading );
        ocp.subjectTo( AT_END  , x4 == 0.0  );
        //ocp.subjectTo( AT_END  , x5  == 0.0  ); // goal ship speed: causes no change to speed
        //ocp.subjectTo( AT_END  , v  == 3.0  );
        ocp.subjectTo( AT_END  , u  == 0.0  );

        ocp.subjectTo( v== 3 );

        
        ocp.subjectTo( -x4max <= x4 <= x4max)   ;
        ocp.subjectTo( -x5max <= x5 <= x5max)   ;
        ocp.subjectTo( -umax <= u <= umax)      ;
        //ocp.subjectTo(  0.0  <= v <= vmax )     ; // speed constraint
        
        
        // VISUALIZE THE RESULTS IN A GNUPLOT WINDOW:
        // ------------------------------------------
        GnuplotWindow window;

        window.addSubplot( x1, x2, "POSITION", "x", "y"     ); // NED
        window.addSubplot( x3 ,     "HEADING" ); 
        window.addSubplot( x4 ,     "STEERING ANGLE" );
        window.addSubplot( x5 ,     "STEERING RATE" ); // NED
        window.addSubplot( v ,      "SPEED" ); // NED
        window.addSubplot( u ,      "STEERING ACCELERATION" ); // NED
        

        //DEFINE AN OPTIMIZATION ALGORITHM AheadingND SOLVE THE OCP:
        // ---------------------------------------------------
        OptimizationAlgorithm algorithm(ocp);

        algorithm.set( MAX_NUM_ITERATIONS, 100);
        algorithm << window;
        algorithm.solve();


        // PRINT THE RESULTS TO TEXT FILES:
        // ---------------------------------------------------
        
        VariablesGrid states;
        algorithm.getDifferentialStates(states); 
        myfile << states ;
        myfile << "\n";

        //myfile << "\n";
        //algorithm.getParameters("primitive_pars.txt");
        //algorithm.getControls("primitive_controls.txt");


        x1.clearStaticCounters();
        x2.clearStaticCounters();
        x3.clearStaticCounters();
        x4.clearStaticCounters();
        x5.clearStaticCounters();
        v.clearStaticCounters() ;
        u.clearStaticCounters() ;
        T.clearStaticCounters() ; 
        
            
    }        
    
    iteration++;
        
    myfile.close();
    return 0;
}
