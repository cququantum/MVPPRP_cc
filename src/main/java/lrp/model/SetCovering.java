package lrp.model;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import java.util.*;

public class SetCovering {

    // --------------------------- instance information & parameter
    /**
     *  customers number include 0;
     */
    public int n;
    /**
     * depot 容量约束
     */
    public double Q;
    /**
     * used in is integer
     */
    public double integral_tolerance = 1e-6;
    /**
     * used in update slack_bounds
     */
    public double slack_values_tolerance = 1e-3;
    /**
     * 每次增加的对偶变量在目标函数中的系数
     */
    public double step_size = 10;

    // --------------------------- columns & solution information
    /**
     * route pool
     */
    public ArrayList<ArrayList<Integer>> column_set;
    /**
     * prevent to repetition
     */
    public HashSet<ArrayList<Integer>> column_hash;
    /**
     * route cost
     */
    public ArrayList<Double> column_cost;
    /**
     * route capacity
     */
    public ArrayList<Double> column_capacity;
    /**
     * current Solution's route/variable id
     */
    public ArrayList<Integer> active_set;
    /**
     * current Solution's route/variable value
     */
    public ArrayList<Double> active_value;
    /**
     * i2ri[i] 表示访问了i点的列的id集合
     */
    public ArrayList<Integer>[] i2ri;
    /**
     * ij2ri[i][j] 表示访问了(i,j)边的列的id集合
     */
    public ArrayList<Integer>[][] ij2ri;

    // --------------------------- results information
    /**
     * objective: solution cost
     */
    public double lpc;
    /**
     * dual of the duration constraint
     */
    public double dur_mu;
    /**
     * dual of the basic constraints
     */
    public double[] mu;
    /**
     * 松弛变量的值
     */
    public double dur_slack_values;
    /**
     * 松弛变量的值
     */
    public double[] slack_values;
    /**
     * duration 松弛变量在目标函数中的系数
     */
    public double dur_slack_bound;
    /**
     * 松弛变量在目标函数中的系数
     */
    public double[] slack_bound;

    // --------------------------- model information
    /**
     * cplex模型
     */
    public IloCplex cplex;
    /**
     * objective constraint
     */
    public IloObjective obj;
    /**
     * routes to variables
     */
    public ArrayList<IloNumVar> column_vars;
    /**
     * duration constraint
     */
    public IloRange dur_range;
    /**
     * constraint
     */
    public IloRange[] ranges;
    /**
     * 每个客户点 对应一个松弛变量
     */
    public IloNumVar[] slack_vars;
    /**
     * duration constraint 对应的 slack variables
     */
    public IloNumVar dur_slack_vars;

    // --------------------------- cuts information
    /**
     * 标记模型中是否有 subset row cuts
     */
    public boolean is_sr = false;
    /**
     * 含有 subset row 的模型
     */
    public SrModel srs;
    /**
     * 含有 capacity cut 的模型
     */
    public CapModel caps;

    /**
     * 空构造函数
     */
    public SetCovering(){}

    /**
     * Initialize a new model
     * @param _n
     * @param _Q
     * @param kl
     * @param ku
     * @param is_stabilization
     * @throws IloException
     */
    public void construct(int _n, double _Q, int kl, int ku, boolean is_stabilization)throws IloException {

        // --------------------------- instance information & parameter
        n = _n;
        Q = _Q;

        // --------------------------- columns & solution information
        column_set = new ArrayList<>();
        column_cost = new ArrayList<>();
        column_capacity = new ArrayList<>();
        column_hash = new HashSet<>();
        active_value = new ArrayList<>();
        active_set = new ArrayList<>();
        i2ri = new ArrayList[n];
        for(int i = 0; i < n; i++){
            i2ri[i] = new ArrayList<>();
        }
        ij2ri = new ArrayList[n+1][n+1];
        for(int i = 0; i < n+1; i++){
            for(int j = 0; j < n+1; j++){
                ij2ri[i][j] = new ArrayList<>();
            }
        }

        // --------------------------- results information
        lpc = 1e10;
        dur_mu = 0;
        mu = new double[n];
        dur_slack_values = 0;
        slack_values = new double[n];
        slack_bound = new double[n];
        if(is_stabilization){
            for(int i = 0; i < n; i++){
                slack_bound[i] = 10;
            }
            dur_slack_bound = 10;
        }else{
            for(int i = 0; i < n; i++){
                slack_bound[i] = 1e6;
            }
            dur_slack_bound = 1e6;
        }

        // --------------------------- model information
        cplex = new IloCplex();
        cplex.setOut(null);                    // 不打印信息
        cplex.setWarning(null);                //
        cplex.setParam(IloCplex.Param.NodeAlgorithm, IloCplex.Algorithm.Primal);   // 设置cplex求解线性松弛模型的算法，单纯型法

        obj = cplex.addMinimize(cplex.linearNumExpr());
        column_vars = new ArrayList<>();

        //-----------construct the cplex model--------------------------------------
        /**
         * depot capacity constraint
         */
        dur_range = cplex.addRange(0, Q,"c.dur");

        ranges = new IloRange[n];
        /**
         * constraints (0)
         */
        ranges[0] = cplex.addRange(kl, ku, "c.0");
        /**
         * constraints (1)
         */
        for(int i = 1; i < n; ++i) {
            ranges[i] = cplex.addRange(1.0, 1e10, "c." + i);
        }

        //----------slacks to cplex---------------------------------------------------
        slack_vars = new IloNumVar[n];
        IloLinearNumExpr obj_expr = cplex.linearNumExpr();
        for(int i = 0; i < n; i++){
            if(ranges[i] != null){
                // 生成松弛变量
                IloNumVar var = cplex.numVar(0.0, 1e10, IloNumVarType.Float, "pos slack," + i);
                // 加入目标函数
                obj_expr.addTerm(slack_bound[i], var);
                // 加入约束
                IloNumExpr expr = ranges[i].getExpr();
                expr = cplex.sum(expr, var);
                // 更新记录的约束
                ranges[i].setExpr(expr);
                // 记录松弛变量
                slack_vars[i] = var;
            }
        }

        // 生成松弛变量
        IloNumVar var = cplex.numVar(0.0, 1e10, IloNumVarType.Float, "dur pos slack");
        // 加入目标函数
        obj_expr.addTerm(dur_slack_bound, var);
        // 加入约束
        IloNumExpr expr = dur_range.getExpr();
        expr = cplex.sum(expr, var);
        // 更新记录的约束
        dur_range.setExpr(expr);
        // 记录松弛变量
        dur_slack_vars = var;

        obj.setExpr(obj_expr);
    }

    /**
     * Solve model
     * @return
     * @throws IloException
     */
    public boolean solve()throws IloException {
        if(cplex.solve()){
            lpc = cplex.getObjValue();
            return true;
        }else{
            throw new IllegalStateException("SetCovering solve failed (infeasible or no solution).");
        }
    }

    /**
     * 添加列信息
     * @param _columns
     * @param _column_cost
     * @param _column_capacity
     * @throws IloException
     */
    public void add_column(ArrayList<ArrayList<Integer>> _columns, ArrayList<Double> _column_cost, ArrayList<Double> _column_capacity)throws IloException {

        for(int i = 0; i < _columns.size(); i++){

            /**
             ------------------------------ 存储加入模型的 columns
             */
            ArrayList<Integer> column = _columns.get(i);
            double cost = _column_cost.get(i);
            double capacity = _column_capacity.get(i);
            column_set.add(column);
            column_hash.add(column);
            column_cost.add(cost);
            column_capacity.add(capacity);

            int place = column_set.size() - 1;

            /**
             ------------------------------ 将 column 加入模型
             */
            // 目标函数部分
            IloColumn col = cplex.column(obj, cost);

            // duration 约束部分
            col = col.and(cplex.column(dur_range, capacity));

            // 约束条件部分
            int[] col_map = new int[n];                      // 数组中被访问的点对应的位置 为 1
            for(int j = 0; j < column.size() - 1; j++){       // 第 0 个位置存储的是 0, 最后一个位置存储的是 0
                int id = column.get(j);
                col_map[id]++;
            }
            for(int j = 0; j < n; j++){
                if(ranges[j] != null)
                    col = col.and(cplex.column(ranges[j], col_map[j]));
            }

            // capacity cut
            if(caps != null){
                col = caps.add_column(col, column);
            }

            // subset row cut 部分
            if(srs != null){
                col = srs.add_column(col, column);
            }

            IloNumVar var = cplex.numVar(col, 0.0,Double.MAX_VALUE, IloNumVarType.Float);

            // 记录新变量
            column_vars.add(var);

            // add to the route map
            for(int j = 0; j < n; j++){
                if(col_map[j] > 0)
                    i2ri[j].add(place);
            }
            for(int j = 0; j < column.size() - 1; j++){
                int id1 = column.get(j);
                int id2 = column.get(j + 1);
                ij2ri[id1][id2].add(place);
            }

        }
    }

    /**
     * 将subset row cut 加入模型
     * @param sr_cuts
     * @throws IloException
     */
    public void add_srcut(ArrayList<ArrayList<Integer>> sr_cuts) throws IloException{
        if(is_sr == false){
            is_sr = true;
            srs = new SrModel(cplex, column_vars, i2ri, n);
        }
        for(int i = 0; i < sr_cuts.size(); i++){
            srs.add_cut(sr_cuts.get(i));
        }
    }

    /**
     * add capacity cut
     * @param lefts
     * @param rights
     * @throws IloException
     */
    public void add_capcut(ArrayList<ArrayList<Integer>> lefts, ArrayList<Integer> rights) throws IloException{
        if(caps == null){
            caps = new CapModel(cplex, column_vars, ij2ri, n);
        }
        for(int i = 0; i < rights.size(); i++){
            caps.add_cut(lefts.get(i), rights.get(i));
        }
    }

    /**
     * 取变量的值
     * @throws IloException
     */
    public boolean set_value()throws IloException {
        // column variables
        active_value.clear();
        active_set.clear();
        for(int i = 0; i < column_set.size(); ++i) {
            IloNumVar var = column_vars.get(i);
            double v = cplex.getValue(var);
            if(v > 0.0000000001){
                active_value.add(v);
                active_set.add(i);
            }
        }

        // slack variables
        dur_slack_values = cplex.getValue(dur_slack_vars);
        for(int i = 0; i < slack_vars.length; i++){
            if(slack_vars[i] != null){
                slack_values[i] = cplex.getValue(slack_vars[i]);
            }
        }

        for(int i = 0; i < slack_values.length; i++){
            if(slack_values[i] > slack_values_tolerance){
                return false;
            }
        }

        // cap cut slack variables
        if(caps != null){
            return caps.set_value();
        }

        return true;
    }

    /**
     * 提取对偶变量
     * @throws IloException
     */
    public void set_dual() throws IloException {
        dur_mu = cplex.getDual(dur_range);
        mu = new double[n];
        for(int i = 0; i < n; i++){
            if(ranges[i] != null)
                mu[i] = cplex.getDual(ranges[i]);
            else
                mu[i] = 0.0;
        }
        if(caps != null){
            caps.get_duals();
        }
        if(srs != null){
            srs.get_duals();
        }
    }

    /**
     * 获取变量值 >= 0.1 且 reduceCost <= 5 的列
     * used in branches
     * @param columns
     * @param costs
     * @throws IloException
     */
    public void get_columns(ArrayList<ArrayList<Integer>> columns, ArrayList<Double> costs, ArrayList<Double> capacities) throws IloException{
        for(int i = 0; i < column_vars.size(); i++){
            IloNumVar var = column_vars.get(i);
            if(var.getUB() < 1e-1)
                continue;
            if(cplex.getReducedCost(var) > 5)
                continue;
            columns.add((ArrayList<Integer>) column_set.get(i).clone());
            costs.add(column_cost.get(i));
            capacities.add(column_capacity.get(i));
        }
    }

    /**
     * 判断该 RMP的解是否为整数解
     * @return
     */
    public boolean is_integral() {
        // check the integrity of each route
        for(int j = 0; j < active_value.size(); j++){
            double v = active_value.get(j);
            int iv = (int) v;
            if(Math.min(v - iv, iv + 1 - v) > integral_tolerance)
                return false;
        }
        return true;
    }

    /**
     * 判断该线性松弛是否可行
     * 通过判断是否所有的松弛变量都等于 0
     * @return
     */
    public boolean is_feasible(){
        for(int i = 0; i < slack_values.length; i++){
            if(Math.abs(slack_values[i]) > 1e-9){
                return false;
            }
        }
        if(Math.abs(dur_slack_values) > 1e-9)
            return false;
        if(caps != null && caps.cap_slack_value != null){
            for(int i = 0; i < caps.cap_slack_value.size(); i++){
                if(Math.abs(caps.cap_slack_value.get(i)) > 1e-9){
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * 更新松弛变量在目标函数中的系数
     * @throws IloException
     */
    public void update_slacks() throws IloException{
        if(slack_vars == null)
            return;
        for(int i = 0; i < slack_values.length; i++) {
            if(slack_values[i] > slack_values_tolerance){
                slack_bound[i] *= step_size;
                cplex.setLinearCoef(obj, slack_vars[i], slack_bound[i]);
            }
        }
        if(dur_slack_values > slack_values_tolerance){
            dur_slack_bound *= step_size;
            cplex.setLinearCoef(obj,dur_slack_vars,dur_slack_bound);
        }
    }

    /**
     * 清除 SetCovering 类所有的参数
     * @throws IloException
     */
    public void clear() throws IloException{
        // --------------------------- instance information & parameter

        // --------------------------- columns & solution information
        if(column_set != null){
            column_set.clear();
            column_set = null;
        }
        if(column_hash != null){
            column_hash.clear();
            column_hash = null;
        }
        if(column_cost != null){
            column_cost.clear();
            column_cost = null;
        }
        if(column_capacity != null){
            column_capacity.clear();
            column_capacity = null;
        }
        if(active_set != null){
            active_set.clear();
            active_set = null;
        }
        if(active_value != null){
            active_value.clear();
            active_value = null;
        }
        for(int i  = 0; i < n + 1; i++){
            for(int j = 0; j < n + 1; j++){
                if(ij2ri[i][j] != null){
                    ij2ri[i][j].clear();
                }
            }
        }
        ij2ri = null;
        for(int i = 0; i < n; i++){
            if(i2ri[i] != null){
                i2ri[i].clear();
            }
        }
        i2ri = null;

        // --------------------------- results information
        lpc = 1e10;
        dur_mu = 0;
        mu = null;
        dur_slack_values = 0;
        slack_values = null;
        dur_slack_bound = 10000000;
        slack_bound = null;

        // --------------------------- model information
        if(cplex != null){
            cplex.clearModel();
            cplex.end();
            cplex = null;
        }
        obj = null;
        column_vars = null;
        dur_range = null;
        ranges = null;
        slack_vars = null;
        dur_slack_vars = null;

        // --------------------------- cuts information
        if(srs != null){
            srs = null;
        }

        if(caps != null){
            caps = null;
        }

    }
}
