import sys; sys.path.insert(0, ".")
import rc_lib as R
print("evaluator's nearest-cell lookup for a sample exactly on each wall face (the lattice line between two cell columns/rows):")
for name, (x, y) in {"E wall (3.0, 0.0125)": (3.0, 0.0125), "W wall (-3.0, 0.0125)": (-3.0, 0.0125), "N wall (0.0125, 3.0)": (0.0125, 3.0), "S wall (0.0125, -3.0)": (0.0125, -3.0)}.items():
    i, j = R.nearest_cell(x, y); cx, cy = R.cell_centre(i, j)
    print(f"  {name}: cell ({i},{j}) centre ({cx:.3f},{cy:.3f}), the {'OUTER' if (abs(cx) > 3 or abs(cy) > 3) else 'INNER'} cell of that wall")
