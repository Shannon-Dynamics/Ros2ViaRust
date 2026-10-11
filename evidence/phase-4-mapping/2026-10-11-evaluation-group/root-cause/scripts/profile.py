import sys, pickle, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
# For each wall, cells across the wall from 3 inside to 3 outside: index lists, with the lattice line between inner1 and outer1.
def cells(wall, m):
    # returns [(i,j)] ordered i3,i2,i1 | o1,o2,o3 for along-wall index m
    if wall == "E": return [(126+k, m) for k in range(6)]          # cols 126..131: inner side is lower col; line between 129|130
    if wall == "W": return [(13-k, m) for k in range(6)]           # cols 13..8: inner is higher col; line between 10|9
    if wall == "N": return [(m, 126+k) for k in range(6)]
    if wall == "S": return [(m, 13-k) for k in range(6)]
for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl","rb"))
    occ = out["L"] >= np.float32(par["occupied_threshold"])
    print("\n==", run, "(pattern: three cells further inside, then |, then [inner cell, outer cell, outer+1 cell] counted from the true face, which lies between inner and outer; X = occupied)")
    for w in "ENWS":
        pat = collections.Counter()
        for m in range(14, 126):     # keep away from the corners
            c = cells(w, m)
            s = "".join("X" if occ[j, i] else "." for i, j in c)
            pat[s[:3] + "|" + s[3:]] += 1
        tot = sum(pat.values())
        print(f"  {w}: " + "  ".join(f"{k}:{v}" for k, v in pat.most_common(7)) + f"   (of {tot} positions)")
