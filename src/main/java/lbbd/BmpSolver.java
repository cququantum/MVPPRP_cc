package lbbd;

import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import instance.Instance;
import model.CplexConfig;
import model.SolveResult;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class BmpSolver {

    private static final double CUT_TOL = 1e-4;
    private static final int MAX_BENDERS_ITER = 500;

    private Instance ins;
    private IloCplex cplex;
    private int cutCounter;
    private Map<String, CvrpSubproblem.Result> subproblemCache;

    // BMP variables
    private IloNumVar[] y;
    private IloNumVar[] p;
    private IloNumVar[] p0;
    private IloNumVar[] i0;
    private IloNumVar[][] z;
    private IloNumVar[][][] lambda;
    private IloNumVar[] omega;

    public SolveResult solve(Instance ins) {
        long startNs = System.nanoTime();
        this.ins = ins;
        this.cutCounter = 0;
        this.subproblemCache = new HashMap<>();

        try (IloCplex cplex = new IloCplex()) {
            this.cplex = cplex;
            configure(cplex);
            buildModel();

            for (int iter = 1; iter <= MAX_BENDERS_ITER; iter++) {
                boolean solved = cplex.solve();
                if (!solved) {
                    return buildResult("LbbdModel", false, cplex, startNs);
                }

                String status = cplex.getStatus().toString();
                if (!status.startsWith("Optimal")) {
                    return buildIncompleteResult("LbbdModel", cplex, startNs,
                            "LBBD incomplete: BMP solve status=" + status);
                }

                CurrentSolution current = extractCurrentSolution();
                SubproblemEvaluation eval = evaluateSubproblems(current);

                if (!eval.allFeasible) {
                    addStrongFeasibilityCut(eval.infeasibleCustomers, eval.infeasibleVBar, eval.infeasiblePeriod);
                    addNoGoodFeasibilityCut(current.lambdaBar, current.lambdaOneCount);
                    continue;
                }

                boolean addedPeriodCut = false;
                boolean addedDualCut = false;
                for (int t = 1; t <= ins.l; t++) {
                    double dualLb = eval.periodResults[t].lpCost;
                    if (current.omegaBar[t] + CUT_TOL < dualLb) {
                        addDualOptimalityCut(t, eval.periodResults[t], current.lambdaBar);
                        addedDualCut = true;
                    }
                }
                for (int t = 1; t <= ins.l; t++) {
                    if (current.omegaBar[t] + CUT_TOL < eval.phiByPeriod[t]) {
                        addPeriodOptimalityCut(t, eval.phiByPeriod[t], current.lambdaBar, current.lambdaOneCountByPeriod[t]);
                        addedPeriodCut = true;
                    }
                }
                if (addedDualCut || addedPeriodCut) {
                    continue;
                }

                if (current.omegaSum + CUT_TOL < eval.phi) {
                    addGlobalOptimalityCut(eval.phi, current.lambdaBar, current.lambdaOneCount);
                    continue;
                }

                return buildResult("LbbdModel", true, cplex, startNs);
            }

            throw new RuntimeException("LBBD reached maximum iterations without convergence: " + MAX_BENDERS_ITER);
        } catch (IloException e) {
            throw new RuntimeException("Failed to solve BmpSolver", e);
        }
    }

    private void buildModel() throws IloException {
        int n = ins.n;
        int l = ins.l;

        y = new IloNumVar[l + 1];
        p = new IloNumVar[l + 1];
        p0 = new IloNumVar[l + 1];
        i0 = new IloNumVar[l + 1];
        omega = new IloNumVar[l + 1];
        z = new IloNumVar[n + 1][l + 1];
        lambda = new IloNumVar[n + 1][l + 2][l + 2];

        for (int t = 1; t <= l; t++) {
            y[t] = cplex.boolVar("y_" + t);
            p[t] = cplex.numVar(0.0, Double.MAX_VALUE, "p_" + t);
            p0[t] = cplex.numVar(0.0, Double.MAX_VALUE, "P0_" + t);
            i0[t] = cplex.numVar(0.0, Double.MAX_VALUE, "I0_" + t);
            omega[t] = cplex.numVar(0.0, Double.MAX_VALUE, "omega_" + t);
        }

        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l; t++) {
                z[i][t] = cplex.boolVar("z_" + i + "_" + t);
            }
        }

        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l + 1; t++) {
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    lambda[i][v][t] = cplex.boolVar("lambda_" + i + "_" + v + "_" + t);
                }
            }
        }

        // Objective
        IloLinearNumExpr obj = cplex.linearNumExpr();
        for (int t = 1; t <= l; t++) {
            obj.addTerm(1.0, omega[t]);
            obj.addTerm(ins.u, p[t]);
            obj.addTerm(ins.f, y[t]);
            obj.addTerm(ins.h0, i0[t]);
            obj.addTerm(ins.hp, p0[t]);
        }
        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l + 1; t++) {
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    obj.addTerm(holdingCostOnArc(ins, i, v, t), lambda[i][v][t]);
                }
            }
        }
        cplex.addMinimize(obj);

        // Constraints (same as ReformulationModelSolver minus routing)
        for (int t = 1; t <= l; t++) {
            IloLinearNumExpr finishedBalance = cplex.linearNumExpr();
            double rhsFinished = ins.dt[t];
            if (t == 1) {
                rhsFinished -= ins.P00;
            } else {
                finishedBalance.addTerm(1.0, p0[t - 1]);
            }
            finishedBalance.addTerm(1.0, p[t]);
            finishedBalance.addTerm(-1.0, p0[t]);
            cplex.addEq(finishedBalance, rhsFinished, "FinishedBalance_" + t);

            IloLinearNumExpr prodCap = cplex.linearNumExpr();
            prodCap.addTerm(1.0, p[t]);
            prodCap.addTerm(-ins.C, y[t]);
            cplex.addLe(prodCap, 0.0, "ProdCap_" + t);

            cplex.addLe(p0[t], ins.Lp, "FinishedCap_" + t);
        }

        for (int t = 1; t <= l; t++) {
            IloLinearNumExpr factoryBalance = cplex.linearNumExpr();
            double rhsFactory = 0.0;
            if (t == 1) {
                rhsFactory = -ins.I00;
            } else {
                factoryBalance.addTerm(1.0, i0[t - 1]);
            }
            for (int i = 1; i <= n; i++) {
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    factoryBalance.addTerm(ins.g(i, v, t), lambda[i][v][t]);
                }
            }
            factoryBalance.addTerm(-ins.k, p[t]);
            factoryBalance.addTerm(-1.0, i0[t]);
            cplex.addEq(factoryBalance, rhsFactory, "FactoryBalance_" + t);

            cplex.addLe(i0[t], ins.L0, "FactoryCap_" + t);
        }

        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l; t++) {
                IloLinearNumExpr link = cplex.linearNumExpr();
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    link.addTerm(1.0, lambda[i][v][t]);
                }
                link.addTerm(-1.0, z[i][t]);
                cplex.addEq(link, 0.0, "LinkZLambda_" + i + "_" + t);
            }
        }

        for (int i = 1; i <= n; i++) {
            IloLinearNumExpr startFlow = cplex.linearNumExpr();
            for (int t = 1; t <= ins.mu[i][0]; t++) {
                if (lambda[i][0][t] != null) {
                    startFlow.addTerm(1.0, lambda[i][0][t]);
                }
            }
            cplex.addEq(startFlow, 1.0, "StartFlow_" + i);
        }

        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l; t++) {
                IloLinearNumExpr balance = cplex.linearNumExpr();
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    balance.addTerm(1.0, lambda[i][v][t]);
                }
                for (int tau = t + 1; tau <= ins.mu[i][t]; tau++) {
                    if (lambda[i][t][tau] != null) {
                        balance.addTerm(-1.0, lambda[i][t][tau]);
                    }
                }
                cplex.addEq(balance, 0.0, "LambdaFlow_" + i + "_" + t);
            }
        }

        for (int i = 1; i <= n; i++) {
            IloLinearNumExpr terminalFlow = cplex.linearNumExpr();
            for (int t = ins.pi[i][l + 1]; t <= l; t++) {
                if (lambda[i][t][l + 1] != null) {
                    terminalFlow.addTerm(1.0, lambda[i][t][l + 1]);
                }
            }
            cplex.addEq(terminalFlow, 1.0, "TerminalFlow_" + i);
        }
    }

    private CurrentSolution extractCurrentSolution() throws IloException {
        int n = ins.n;
        int l = ins.l;

        CurrentSolution current = new CurrentSolution();
        current.zBar = new double[n + 1][l + 1];
        current.lambdaBar = new double[n + 1][l + 2][l + 2];
        current.omegaBar = new double[l + 1];
        current.lambdaOneCountByPeriod = new int[l + 1];

        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l; t++) {
                current.zBar[i][t] = cplex.getValue(z[i][t]);
            }
        }
        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l + 1; t++) {
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    if (lambda[i][v][t] != null) {
                        current.lambdaBar[i][v][t] = cplex.getValue(lambda[i][v][t]);
                    }
                }
            }
        }
        for (int t = 1; t <= l; t++) {
            current.omegaBar[t] = cplex.getValue(omega[t]);
            current.omegaSum += current.omegaBar[t];
        }

        int oneCount = 0;
        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l; t++) {
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    if (current.lambdaBar[i][v][t] > 0.5) {
                        oneCount++;
                        current.lambdaOneCountByPeriod[t]++;
                    }
                }
            }
        }
        current.lambdaOneCount = oneCount;

        return current;
    }

    private SubproblemEvaluation evaluateSubproblems(CurrentSolution current) throws IloException {
        int l = ins.l;
        SubproblemEvaluation eval = new SubproblemEvaluation();
        eval.allFeasible = true;
        eval.phi = 0.0;
        eval.phiByPeriod = new double[l + 1];
        eval.periodResults = new CvrpSubproblem.Result[l + 1];

        for (int t = 1; t <= l; t++) {
            String cacheKey = periodPatternKey(current.lambdaBar, t);
            CvrpSubproblem.Result exact = subproblemCache.get(cacheKey);
            if (exact == null) {
                exact = CvrpSubproblem.solveExact(ins, t, current.zBar, current.lambdaBar);
                subproblemCache.put(cacheKey, copySubproblemResult(exact));
            } else {
                exact = copySubproblemResult(exact);
            }
            eval.periodResults[t] = exact;
            if (!exact.feasible) {
                eval.allFeasible = false;
                eval.infeasiblePeriod = t;
                eval.infeasibleCustomers = exact.activeCust;
                eval.infeasibleVBar = exact.vBar;
                return eval;
            }
            eval.phiByPeriod[t] = exact.optCost;
            eval.phi += exact.optCost;
        }
        return eval;
    }

    private String periodPatternKey(double[][][] lambdaBar, int period) {
        StringBuilder sb = new StringBuilder(16 + ins.n * 4);
        sb.append(period).append('|');
        for (int i = 1; i <= ins.n; i++) {
            int chosenV = -1;
            for (int v = ins.pi[i][period]; v <= period - 1; v++) {
                if (lambdaBar[i][v][period] > 0.5) {
                    chosenV = v;
                    break;
                }
            }
            sb.append(chosenV).append(',');
        }
        return sb.toString();
    }

    private static CvrpSubproblem.Result copySubproblemResult(CvrpSubproblem.Result src) {
        CvrpSubproblem.Result dst = new CvrpSubproblem.Result();
        dst.feasible = src.feasible;
        dst.optCost = src.optCost;
        dst.lpCost = src.lpCost;
        dst.dur_mu = src.dur_mu;
        dst.mu = (src.mu == null) ? null : src.mu.clone();
        dst.vBar = (src.vBar == null) ? null : src.vBar.clone();
        dst.activeCust = (src.activeCust == null) ? null : new ArrayList<>(src.activeCust);
        return dst;
    }

    private void addStrongFeasibilityCut(List<Integer> activeCustomers, int[] vBar, int period) throws IloException {
        IloLinearNumExpr lhs = cplex.linearNumExpr();
        for (int i : activeCustomers) {
            int v = vBar[i];
            IloNumVar var = lambda[i][v][period];
            if (var != null) {
                lhs.addTerm(-1.0, var);
            }
        }
        double rhs = 1.0 - activeCustomers.size();
        cplex.addGe(lhs, rhs, nextCutName("FeasStrong"));
    }

    private void addNoGoodFeasibilityCut(double[][][] lambdaBar, int lambdaOneCount) throws IloException {
        int n = ins.n;
        int l = ins.l;
        IloLinearNumExpr lhs = cplex.linearNumExpr();
        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l; t++) {
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    IloNumVar var = lambda[i][v][t];
                    if (var == null) {
                        continue;
                    }
                    if (lambdaBar[i][v][t] > 0.5) {
                        lhs.addTerm(-1.0, var);
                    } else {
                        lhs.addTerm(1.0, var);
                    }
                }
            }
        }
        double rhs = 1.0 - lambdaOneCount;
        cplex.addGe(lhs, rhs, nextCutName("FeasNoGood"));
    }

    private void addGlobalOptimalityCut(double phi, double[][][] lambdaBar, int lambdaOneCount) throws IloException {
        int n = ins.n;
        int l = ins.l;
        IloLinearNumExpr lhs = cplex.linearNumExpr();
        for (int t = 1; t <= l; t++) {
            lhs.addTerm(1.0, omega[t]);
        }
        for (int i = 1; i <= n; i++) {
            for (int t = 1; t <= l; t++) {
                for (int v = ins.pi[i][t]; v <= t - 1; v++) {
                    IloNumVar var = lambda[i][v][t];
                    if (var == null) {
                        continue;
                    }
                    if (lambdaBar[i][v][t] > 0.5) {
                        lhs.addTerm(-phi, var);
                    } else {
                        lhs.addTerm(phi, var);
                    }
                }
            }
        }
        double rhs = phi - phi * lambdaOneCount;
        cplex.addGe(lhs, rhs, nextCutName("OptGlobal"));
    }

    private void addDualOptimalityCut(int period, CvrpSubproblem.Result sp, double[][][] lambdaBar) throws IloException {
        IloLinearNumExpr lhs = cplex.linearNumExpr();
        lhs.addTerm(1.0, omega[period]);

        double rhs = ins.K * sp.dur_mu;
        for (int i : sp.activeCust) {
            double ui = sp.mu[i];
            int v = sp.vBar[i];
            IloNumVar var = lambda[i][v][period];
            if (var != null) {
                lhs.addTerm(-ui, var);
            }
        }

        cplex.addGe(lhs, rhs, nextCutName("OptDual_" + period));
    }

    private void addPeriodOptimalityCut(int period, double periodPhi, double[][][] lambdaBar, int lambdaOneCountAtPeriod) throws IloException {
        int n = ins.n;
        IloLinearNumExpr lhs = cplex.linearNumExpr();
        lhs.addTerm(1.0, omega[period]);
        for (int i = 1; i <= n; i++) {
            for (int v = ins.pi[i][period]; v <= period - 1; v++) {
                IloNumVar var = lambda[i][v][period];
                if (var != null) {
                    if (lambdaBar[i][v][period] > 0.5) {
                        lhs.addTerm(-periodPhi, var);
                    } else {
                        lhs.addTerm(periodPhi, var);
                    }
                }
            }
        }
        double rhs = periodPhi - periodPhi * lambdaOneCountAtPeriod;
        cplex.addGe(lhs, rhs, nextCutName("OptPeriod_" + period));
    }

    private String nextCutName(String prefix) {
        cutCounter++;
        return prefix + "_" + cutCounter;
    }

    private static double holdingCostOnArc(Instance ins, int i, int v, int t) {
        return ins.e(i, v, t);
    }

    private static void configure(IloCplex cplex) throws IloException {
        cplex.setParam(IloCplex.Param.TimeLimit, CplexConfig.TIME_LIMIT_SEC);
        cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, CplexConfig.MIP_GAP);
        if (!CplexConfig.LOG_TO_CONSOLE) {
            cplex.setOut(null);
            cplex.setWarning(null);
        }
    }

    private static SolveResult buildResult(String modelName, boolean solved, IloCplex cplex, long startNs) throws IloException {
        double sec = (System.nanoTime() - startNs) / 1_000_000_000.0;
        String status = cplex.getStatus().toString();
        boolean optimal = status.startsWith("Optimal");
        double objective = Double.NaN;
        if (solved) {
            objective = cplex.getObjValue();
        }
        double bestBound = safeBestBound(cplex);
        double gap = safeGap(cplex);
        return new SolveResult(modelName, solved, optimal, status, objective, bestBound, gap, sec);
    }

    private static SolveResult buildIncompleteResult(String modelName, IloCplex cplex, long startNs, String statusText) throws IloException {
        double sec = (System.nanoTime() - startNs) / 1_000_000_000.0;
        double objective = Double.NaN;
        try {
            objective = cplex.getObjValue();
        } catch (IloException ignored) {
        }
        double bestBound = safeBestBound(cplex);
        double gap = safeGap(cplex);
        return new SolveResult(modelName, true, false, statusText, objective, bestBound, gap, sec);
    }

    private static double safeBestBound(IloCplex cplex) {
        try {
            return cplex.getBestObjValue();
        } catch (IloException e) {
            return Double.NaN;
        }
    }

    private static double safeGap(IloCplex cplex) {
        try {
            return cplex.getMIPRelativeGap();
        } catch (IloException e) {
            return Double.NaN;
        }
    }

    private static final class CurrentSolution {
        double[][] zBar;
        double[][][] lambdaBar;
        double[] omegaBar;
        int lambdaOneCount;
        int[] lambdaOneCountByPeriod;
        double omegaSum;
    }

    private static final class SubproblemEvaluation {
        boolean allFeasible;
        double phi;
        double[] phiByPeriod;
        CvrpSubproblem.Result[] periodResults;
        int infeasiblePeriod;
        List<Integer> infeasibleCustomers;
        int[] infeasibleVBar;
    }

}
