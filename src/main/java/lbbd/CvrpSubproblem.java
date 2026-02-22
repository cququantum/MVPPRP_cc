package lbbd;

import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;
import instance.Instance;
import model.CplexConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CvrpSubproblem {

    private static final double EPS = 1e-9;
    private static final double INF = 1e100;

    public static class Result {
        public boolean feasible;
        public double optCost;
        public double lpCost;
        public double[] mu;     // duals for customer cover constraints (global customer index)
        public double dur_mu;   // dual for vehicle-count constraint (u_0 in lbbd.tex notation)
        public List<Integer> activeCust;
        public int[] vBar;
    }

    private static final class RoutePool {
        int[] masks;        // bitmask over local active-customer indices 0..m-1
        double[] costs;     // shortest depot-tour cost for each mask
    }

    private static final class SpSolveResult {
        boolean feasible;
        String status;
        double objective;
        double[] coverDuals;
        double vehicleDual;
    }

    public static Result solveExact(Instance ins, int t, double[][] zBar, double[][][] lambdaBar) throws IloException {
        List<Integer> active = new ArrayList<>();
        for (int i = 1; i <= ins.n; i++) {
            if (zBar[i][t] > 0.5) {
                active.add(i);
            }
        }

        int[] vBarArr = new int[ins.n + 1];
        for (int i : active) {
            for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                if (lambdaBar[i][v][t] > 0.5) {
                    vBarArr[i] = v;
                    break;
                }
            }
        }

        Result res = new Result();
        res.activeCust = active;
        res.vBar = vBarArr;
        res.mu = new double[ins.n + 1];
        res.dur_mu = 0.0;

        if (active.isEmpty()) {
            res.feasible = true;
            res.optCost = 0.0;
            res.lpCost = 0.0;
            return res;
        }

        int m = active.size();
        int[] localToGlobal = new int[m];
        double[] demand = new double[m];
        for (int idx = 0; idx < m; idx++) {
            int global = active.get(idx);
            localToGlobal[idx] = global;
            demand[idx] = ins.g(global, vBarArr[global], t);
            if (demand[idx] > ins.Q + EPS) {
                res.feasible = false;
                res.optCost = Double.POSITIVE_INFINITY;
                res.lpCost = Double.POSITIVE_INFINITY;
                return res;
            }
        }

        if (m >= Integer.SIZE - 1) {
            solveExactFallbackArcModel(ins, localToGlobal, demand, res);
            return res;
        }

        RoutePool routes = buildRoutePool(ins, localToGlobal, demand);
        if (routes.masks.length == 0) {
            res.feasible = false;
            res.optCost = Double.POSITIVE_INFINITY;
            res.lpCost = Double.POSITIVE_INFINITY;
            return res;
        }

        SpSolveResult exact = solveSetPartitioning(routes, m, ins.K, true);
        if (!exact.feasible) {
            res.feasible = false;
            res.optCost = Double.POSITIVE_INFINITY;
            res.lpCost = Double.POSITIVE_INFINITY;
            return res;
        }

        SpSolveResult lp = solveSetPartitioning(routes, m, ins.K, false);
        if (!lp.feasible) {
            throw new IloException("LP relaxation of route set-partitioning subproblem is infeasible unexpectedly.");
        }

        res.feasible = true;
        res.optCost = exact.objective;
        res.lpCost = lp.objective;
        res.dur_mu = lp.vehicleDual;
        for (int idx = 0; idx < m; idx++) {
            res.mu[localToGlobal[idx]] = lp.coverDuals[idx];
        }
        return res;
    }

    private static void solveExactFallbackArcModel(Instance ins, int[] localToGlobal, double[] demand, Result res) throws IloException {
        int m = localToGlobal.length;
        int nodeCount = m + 2; // 0=start depot, 1..m=customers, m+1=end depot
        int end = m + 1;

        try (IloCplex cplex = new IloCplex()) {
            cplex.setParam(IloCplex.Param.TimeLimit, CplexConfig.TIME_LIMIT_SEC);
            cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, CplexConfig.MIP_GAP);
            cplex.setOut(null);
            cplex.setWarning(null);

            IloNumVar[][] x = new IloNumVar[nodeCount][nodeCount];
            IloNumVar[] u = new IloNumVar[nodeCount];

            for (int i = 0; i < nodeCount; i++) {
                u[i] = cplex.numVar(0.0, ins.Q, "u_" + i);
            }
            cplex.addEq(u[0], 0.0, "u_depot");

            for (int i = 0; i < nodeCount; i++) {
                for (int j = 0; j < nodeCount; j++) {
                    if (i == j) {
                        continue;
                    }
                    x[i][j] = cplex.boolVar("x_" + i + "_" + j);
                    if (!isLocalRoutingArc(i, j, end)) {
                        cplex.addEq(x[i][j], 0.0, "InvalidArc_" + i + "_" + j);
                    }
                }
            }

            IloLinearNumExpr obj = cplex.linearNumExpr();
            for (int i = 0; i < nodeCount; i++) {
                for (int j = 0; j < nodeCount; j++) {
                    if (i == j || x[i][j] == null) {
                        continue;
                    }
                    if (!isLocalRoutingArc(i, j, end)) {
                        continue;
                    }
                    obj.addTerm(localArcCost(ins, localToGlobal, i, j, end), x[i][j]);
                }
            }
            cplex.addMinimize(obj);

            for (int c = 1; c <= m; c++) {
                IloLinearNumExpr in = cplex.linearNumExpr();
                IloLinearNumExpr out = cplex.linearNumExpr();
                for (int j = 0; j < nodeCount; j++) {
                    if (j != c && x[j][c] != null) {
                        in.addTerm(1.0, x[j][c]);
                    }
                    if (j != c && x[c][j] != null) {
                        out.addTerm(1.0, x[c][j]);
                    }
                }
                cplex.addEq(in, 1.0, "CoverIn_" + c);
                cplex.addEq(out, 1.0, "CoverOut_" + c);
            }

            IloLinearNumExpr depart = cplex.linearNumExpr();
            IloLinearNumExpr back = cplex.linearNumExpr();
            for (int c = 1; c <= m; c++) {
                depart.addTerm(1.0, x[0][c]);
                back.addTerm(1.0, x[c][end]);
            }
            cplex.addEq(depart, back, "RouteCountBalance");
            cplex.addLe(depart, ins.K, "RouteCountLimit");

            double bigM = Math.max(ins.bigM, ins.Q);
            for (int i = 0; i < nodeCount; i++) {
                for (int j = 0; j < nodeCount; j++) {
                    if (i == j || x[i][j] == null || !isLocalRoutingArc(i, j, end)) {
                        continue;
                    }
                    IloLinearNumExpr mtz = cplex.linearNumExpr();
                    mtz.addTerm(1.0, u[j]);
                    mtz.addTerm(-1.0, u[i]);
                    mtz.addTerm(-bigM, x[i][j]);
                    double rhs = -bigM;
                    if (i >= 1 && i <= m) {
                        rhs += demand[i - 1];
                    }
                    cplex.addGe(mtz, rhs, "MTZ_" + i + "_" + j);
                }
            }

            boolean solved = cplex.solve();
            String status = cplex.getStatus().toString();
            if (status.startsWith("Optimal")) {
                res.feasible = true;
                res.optCost = cplex.getObjValue();
                // No dual information from this fallback; disable dual-cut generation for this period.
                res.lpCost = Double.NaN;
                res.dur_mu = 0.0;
                return;
            }
            if (status.startsWith("Infeasible")) {
                res.feasible = false;
                res.optCost = Double.POSITIVE_INFINITY;
                res.lpCost = Double.POSITIVE_INFINITY;
                return;
            }
            throw new IloException("CVRP fallback subproblem not solved to proven optimality: " + status + ", solved=" + solved);
        }
    }

    private static boolean isLocalRoutingArc(int i, int j, int end) {
        if (i == j) {
            return false;
        }
        if (i == end) {
            return false;
        }
        if (j == 0) {
            return false;
        }
        return !(i == 0 && j == end);
    }

    private static double localArcCost(Instance ins, int[] localToGlobal, int i, int j, int end) {
        int gi = (i == 0) ? 0 : (i == end ? ins.n + 1 : localToGlobal[i - 1]);
        int gj = (j == 0) ? 0 : (j == end ? ins.n + 1 : localToGlobal[j - 1]);
        return ins.c[gi][gj];
    }

    private static RoutePool buildRoutePool(Instance ins, int[] localToGlobal, double[] demand) {
        int m = localToGlobal.length;
        int subsetCount = 1 << m;

        double[] demandSum = new double[subsetCount];
        double[] subsetRouteCost = new double[subsetCount];
        Arrays.fill(subsetRouteCost, INF);

        double[][] dp = new double[subsetCount][m];
        for (int mask = 0; mask < subsetCount; mask++) {
            Arrays.fill(dp[mask], INF);
        }

        for (int j = 0; j < m; j++) {
            int mask = 1 << j;
            int gj = localToGlobal[j];
            dp[mask][j] = ins.c[0][gj];
        }

        for (int mask = 1; mask < subsetCount; mask++) {
            int lsb = Integer.numberOfTrailingZeros(mask);
            int prev = mask & (mask - 1);
            demandSum[mask] = demandSum[prev] + demand[lsb];

            if (prev != 0) {
                for (int j = 0; j < m; j++) {
                    if ((mask & (1 << j)) == 0) {
                        continue;
                    }
                    int prevMask = mask ^ (1 << j);
                    if (prevMask == 0) {
                        continue;
                    }
                    int gj = localToGlobal[j];
                    double best = INF;
                    for (int k = 0; k < m; k++) {
                        if ((prevMask & (1 << k)) == 0) {
                            continue;
                        }
                        int gk = localToGlobal[k];
                        double cand = dp[prevMask][k] + ins.c[gk][gj];
                        if (cand < best) {
                            best = cand;
                        }
                    }
                    dp[mask][j] = best;
                }
            }

            if (demandSum[mask] <= ins.Q + EPS) {
                double bestRoute = INF;
                for (int j = 0; j < m; j++) {
                    if ((mask & (1 << j)) == 0) {
                        continue;
                    }
                    int gj = localToGlobal[j];
                    double cand = dp[mask][j] + ins.c[gj][ins.n + 1];
                    if (cand < bestRoute) {
                        bestRoute = cand;
                    }
                }
                subsetRouteCost[mask] = bestRoute;
            }
        }

        List<Integer> maskList = new ArrayList<>();
        List<Double> costList = new ArrayList<>();
        for (int mask = 1; mask < subsetCount; mask++) {
            if (subsetRouteCost[mask] < INF / 2) {
                maskList.add(mask);
                costList.add(subsetRouteCost[mask]);
            }
        }

        RoutePool pool = new RoutePool();
        pool.masks = new int[maskList.size()];
        pool.costs = new double[costList.size()];
        for (int r = 0; r < maskList.size(); r++) {
            pool.masks[r] = maskList.get(r);
            pool.costs[r] = costList.get(r);
        }
        return pool;
    }

    private static SpSolveResult solveSetPartitioning(RoutePool routes, int customerCount, int maxVehicles, boolean integral) throws IloException {
        try (IloCplex cplex = new IloCplex()) {
            cplex.setParam(IloCplex.Param.TimeLimit, CplexConfig.TIME_LIMIT_SEC);
            cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, CplexConfig.MIP_GAP);
            cplex.setOut(null);
            cplex.setWarning(null);

            int routeCount = routes.masks.length;
            IloNumVar[] xi = new IloNumVar[routeCount];
            for (int r = 0; r < routeCount; r++) {
                xi[r] = integral ? cplex.boolVar("xi_" + r) : cplex.numVar(0.0, 1.0, "xi_" + r);
            }

            IloLinearNumExpr obj = cplex.linearNumExpr();
            for (int r = 0; r < routeCount; r++) {
                obj.addTerm(routes.costs[r], xi[r]);
            }
            cplex.addMinimize(obj);

            IloRange[] cover = new IloRange[customerCount];
            for (int j = 0; j < customerCount; j++) {
                IloLinearNumExpr expr = cplex.linearNumExpr();
                int bit = 1 << j;
                for (int r = 0; r < routeCount; r++) {
                    if ((routes.masks[r] & bit) != 0) {
                        expr.addTerm(1.0, xi[r]);
                    }
                }
                cover[j] = cplex.addEq(expr, 1.0, "Cover_" + j);
            }

            IloLinearNumExpr vehExpr = cplex.linearNumExpr();
            for (int r = 0; r < routeCount; r++) {
                vehExpr.addTerm(1.0, xi[r]);
            }
            IloRange veh = cplex.addLe(vehExpr, maxVehicles, "VehLimit");

            SpSolveResult ret = new SpSolveResult();
            boolean solved = cplex.solve();
            String status = cplex.getStatus().toString();
            boolean optimal = status.startsWith("Optimal");
            ret.status = status;

            if (optimal) {
                ret.feasible = true;
            } else if (status.startsWith("Infeasible")) {
                ret.feasible = false;
            } else {
                throw new IloException("CVRP subproblem not solved to proven optimality (integral=" + integral + "): " + status);
            }

            if (!ret.feasible) {
                ret.objective = Double.POSITIVE_INFINITY;
                return ret;
            }

            ret.objective = cplex.getObjValue();
            if (!integral) {
                ret.coverDuals = new double[customerCount];
                for (int j = 0; j < customerCount; j++) {
                    ret.coverDuals[j] = cplex.getDual(cover[j]);
                }
                ret.vehicleDual = cplex.getDual(veh);
            }
            return ret;
        }
    }
}
