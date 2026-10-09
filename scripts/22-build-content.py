# Runs both content generators in ONE editor launch. The initial asset scan
# dominates the cost of a commandlet, so paying it twice is pure waste.
# Material first: the level places a vehicle whose mirrors look for it.
import os
HERE = os.path.dirname(os.path.abspath(__file__))
for s in ("21-make-mirror-material.py", "20-make-level.py"):
    p = os.path.join(HERE, s)
    print("[build-content] running %s" % p)
    exec(compile(open(p).read(), p, "exec"), {"__name__": "__main__"})
print("[build-content] done")
