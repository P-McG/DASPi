#!/bin/bash

increment_subnet() {
    local ip="$1"
    IFS='.' read -r o1 o2 o3 o4 <<< "$ip"
    
    ((o3++))
    if ((o3 > 255)); then
        echo "Error: Subnet (3rd octet) overflow"
        return 1
    fi

    echo "$o1.$o2.$o3.$o4"
}

HOSTS=()
PORT=5000
CLIENT="10.0.2.2"

if [ "$#" -eq 0 ]; then
   HOSTS=("aperturecomputemodule000" "aperturecomputemodule001") # Replace with your actual hosts
else
   HOSTS=("$@") # Remaining arguments are treated as the host list
fi

# Loop through the hosts
for HOST in "${HOSTS[@]}"; do
    echo "Connecting to $HOST as $USER on port $PORT..."

    # Execute a remote command via SSH
    ssh "$USER@$HOST" "echo 'Starting $HOST on port $PORT'"

    # Use SSH to copy over the files.
    ssh -t "$USER@$HOST" "rm -r -f ~/DASPi/aperturecomputemodule"
    scp -r ~/DASPi/aperturecomputemodule "$USER@$HOST:~/DASPi/aperturecomputemodule"

    # Start the process
    ssh -t "$USER@$HOST" "tmux new-session -d -s trial_1 'ulimit -s unlimited; \
        ~/DASPi/aperturecomputemodule/bin/aperturecomputemodule \
        --verbose --clientIp=$CLIENT --port=$PORT > trial.log 2>&1'"
    
    CLIENT=$(increment_subnet "$CLIENT")

done

