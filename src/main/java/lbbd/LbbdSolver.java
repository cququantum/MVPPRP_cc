package lbbd;

import instance.Instance;
import model.SolveResult;

public final class LbbdSolver {

    public SolveResult solve(Instance ins) {
        return new BmpSolver().solve(ins);
    }
}
