# This run is incomplete, kept because it is what happened

Two separate problems, both mine, neither in the stack under test.

1. **Hang after K10.** The first version of section K stopped its second MCL with
   `kill -INT $PID; wait $PID`. A plain background job of a non-interactive shell has SIGINT
   ignored, so the signal did nothing and `wait` blocked for ten minutes. The node was
   killed by hand to unblock it. Every K check up to K10 had already passed and is in
   `verify.log`. The fix is `setsid` plus a signal to the process group, as the launch
   already did, and a new check K11 that the node does exit on SIGINT.
2. **The script was edited while it ran.** Bash reads a script incrementally, so applying
   that fix to `scripts/verify_phase3_localization.sh` mid-run corrupted the rest of the
   run: it died with a syntax error at the shutdown section, after the launch had exited.
   There is therefore no `summary.yaml` and no verdict for this directory.

Do not read a pass count from this directory. The run that counts is the one that was
started after the fix, in the sibling directory without this note.
