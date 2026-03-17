#!/bin/bash
#distribute_and_run_aperturecomputemodule.sh

set -e

increment_subnet() {
    local ip="$1"
    IFS='.' read -r o1 o2 o3 o4 <<< "$ip"

    ((o3++))
    if ((o3 > 255)); then
        echo "Error: Subnet (3rd octet) overflow" >&2
        return 1
    fi

    echo "$o1.$o2.$o3.$o4"
}

HOSTS=()
PORT=5000
CLIENT="10.0.2.2"

if [ "$#" -eq 0 ]; then
    HOSTS=("aperturecomputemodule000" "aperturecomputemodule001")
else
    HOSTS=("$@")
fi

for HOST in "${HOSTS[@]}"; do
    mkdir -p ~/DASPi/src/bin-aperturecomputemodule/aperturecomputemodule
    cp ~/DASPi/src/build-aperturecomputemodule/aperturecomputemodule/aperturecomputemodule ~/DASPi/src/bin-aperturecomputemodule/aperturecomputemodule

    echo "Connecting to $HOST as $USER, client IP $CLIENT, port $PORT"
    ssh "$USER@$HOST" "
        rm -rf ~/DASPi/src/bin-aperturecomputemodule &&
        mkdir -p ~/DASPi/src/bin-aperturecomputemodule/aperturecomputemodule
    "
    scp -r ~/DASPi/src/bin-aperturecomputemodule/aperturecomputemodule \
        "$USER@$HOST:~/DASPi/src/bin-aperturecomputemodule"
        
    ssh "$USER@$HOST" "chmod +x ~/DASPi/src/bin-aperturecomputemodule/aperturecomputemodule/aperturecomputemodule"

    ssh "$USER@$HOST" "
        tmux kill-session -t aperturecomputemodule 2>/dev/null || true
        tmux new-session -d -s aperturecomputemodule '
            ulimit -s unlimited
            cd ~/DASPi/src/bin-aperturecomputemodule/aperturecomputemodule
            exec ./aperturecomputemodule \
                --verbose \
                --clientIp=$CLIENT \
                --port=$PORT \
                > aperturecomputemodule.log 2>&1
        '
    "

    CLIENT=$(increment_subnet "$CLIENT")
done
