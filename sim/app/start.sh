#!/bin/sh
# Starts the simulator server. Open http://localhost:5173 afterwards.
cd "$(dirname "$0")" || exit 1
export PATH="$PWD/../.node/bin:$PATH"
command -v node >/dev/null || { echo "Node 20+ is not installed. Get it from https://nodejs.org, then run this again."; exit 1; }
[ -d ../physics/node_modules ] || (cd ../physics && npm install --no-audit --no-fund)
[ -d node_modules ] || npm install --no-audit --no-fund
exec node server.mjs
