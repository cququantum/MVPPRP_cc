package lrp.data;

public class Parameter {

    // instance
    public int MaxVehicleNumber = (int)Double.MAX_VALUE;
    public double Tolerance = 1e-6;                   // high precision
    public double dualStabilization = 1e6;            // for dual stabilization

    // cuts
    public boolean isAddBendersCut = true;              // 是否添加benders cut

    // print
    public boolean isPrintPBiter = true;              // is print BPTest process
    public boolean isPrintBranchArc = true;          // is print branch arc process
    public boolean PrintBendersAssign = true;         // is print assign Solution
    public boolean isPrintMP = true;                  // is print master Problem information
    public boolean isPrintSP = false;                  // print BD subProblem
    public boolean isPrintIter = true;                // print BD iter information
    public boolean isPrintCuts = false;                // is print cuts
    public boolean isPrintLabelsetting = true;        // print BD subProblem
    public boolean isPrintHeuristic = false;           // print heuristic

    // results parameter
    public double start_time;                         // 从读取数据开始
    public double time_limit = 20;                  // 时间限制
    public int node_num_bp;                           // BP 搜寻的 node 数量

    // ----------------- huristic

    // Problem parameters
    public int SEED = 1;
    public int MAX_ROUTE_LENGTH = 200;
    public int MAX_OPT_SIZE = 8;
    public boolean MULTI_TRIP_ENABLE = false;
    public boolean CAPACITY_CONSTRAINT_ADD = true;
    public boolean CAPACITY_CONSTRAINT_RELAX = false;
    public boolean DURATION_CONSTRAINT_ADD = false;
    public boolean DURATION_CONSTRAINT_RELAX = false;
    public boolean TIMEWINDOW_CONSTRAINT_ADD = false;
    public boolean TIMEWINDOW_CONSTRAINT_RELAX = false;

    // Tabu search parameter
    public int TABU_MAX_ITERATION = 10000;
    public int TABU_TIME_LIMIT = 3600;
    public int INIT_TABU_TENURE = 30;
    public double TABU_DECAY_RATE = 0.98;
    public int MIN_TABU_TENURE = 10;
    public int INIT_SHAKE_TENURE = 100;
    public int SHAKE_TENURE_INCREMENT = 100;
    public int MAX_SHAKE_TENURE = 200;
    public int INIT_SHAKE_ITERATION  = 10;
    public int SHAKE_ITERATION_INCREMENT = 0;
    public int MAX_SHAKE_ITERATION = 50;
    public boolean TABU_FEASIBLE_BREAK = false;
    public boolean TABU_LOG_PRINT = false;
    public int TABU_LOG_TENURE = 1;
    public boolean TABU_LOG_DETAIL = false;
    public boolean TABU_RELOCATE_ADD = true;  // if the objective is to minimize the number of vehicles, use relocate only; otherwise use all the three operators
    public boolean TABU_EXCHANGE_ADD = true;
    public boolean TABU_CROSS_ADD = true;

    // Constraints relaxation penalty coefficient
    public double CAPACITY_PENALTY = 100;
    public double DURATION_PENALTY = 100;
    public double TIMEWINDOW_PENALTY = 50;

    // Local search parameter
    public boolean LOCALSEARCH_LOG_PRINT = false;

    // Initial Solution Parameter
    public boolean INITIAL_LOG_PRINT = false;
    public int INITIAL_EMPTY_ADD = 1;

    // Neighborhood reduction parameters
    public double INSERTION_PRUNE_THRESHHOLD = 10;
    public double CROSS_PRUNE_THRESHHOLD = 10;
    public double EXCHANGE_PRUNE_THRESHHOLD = 10;
}
