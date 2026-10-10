# Failed at start-up, superseded, not graded

This run wrote `environment.txt`, `run.log` and `launch.log` and no trial. `launch.log` ends
with the TurtleBot3 driver retrying "The specified robot ... is not in the list of robots
with <extern> controllers" while the launch was being shut down. The directory was written
into the WSL mirror rather than the Windows checkout (so `scripts/evidence_root.sh` did not see
the checkout as writable at that moment) and was copied here by hand afterwards, unchanged.

Not used in any table. The final replicates are the `...-final-...` runs. Kept, as every run
is.
