import pickle, numpy as np
d = pickle.load(open("warp.pkl", "rb"))
B = np.concatenate([v[0] for v in d.values()]); T = np.concatenate([v[1] for v in d.values()])
RES = np.concatenate([v[2] for v in d.values()]); AL = np.concatenate([v[3] for v in d.values()])
m = np.abs(np.degrees(AL)) > 10; x = T * np.tan(AL)
slope = 2 * np.pi / 359 - 2 * np.pi / 360
print("pooled over the four runs. eps(i) in mrad per beam index i, fitted from range residuals; 'line' = i * (2pi/359 - 2pi/360) = i * %.2e rad" % slope)
print("first 20 beams individually, then 10-beam groups; n = wall beams used\n")
def fit(sel):
    if sel.sum() < 80: return None, int(sel.sum())
    return (RES[sel] * x[sel]).sum() / (x[sel] ** 2).sum() * 1000, int(sel.sum())
print("beam   eps   line     n")
for i in range(0, 20):
    e, n = fit(m & (B == i)); print(f"{i:4d}  " + (f"{e:+5.1f}" if e is not None else "   --") + f"  {i*slope*1000:+5.1f}  {n:5d}")
print("\ngroup of 10     eps   line      n")
for g in range(2, 36):
    lo, hi = g * 10, g * 10 + 10
    e, n = fit(m & (B >= lo) & (B < hi)); print(f"{lo:4d}-{hi-1:<4d}      " + (f"{e:+5.1f}" if e is not None else "   --") + f"  {(lo+4.5)*slope*1000:+5.1f}  {n:6d}")
