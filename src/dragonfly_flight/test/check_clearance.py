"""Offline conservative envelope check; unsafe paths are rejected at runtime."""
import json
import sys
from dragonfly_flight.clearance import check

if __name__=='__main__':
    failures=check(sys.argv[1],float(sys.argv[2]) if len(sys.argv)>2 else 10)
    print(json.dumps({'intersections':len(failures),'examples':failures[:6]},indent=2))
    sys.exit(bool(failures))
