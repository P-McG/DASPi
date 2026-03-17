#!/bin/bash

SESSION="camera_dashboard"
USER="${USER:-pi}"
REMOTE_LOG="~/DASPi/software/bin/aperturecomputemodule/aperturecomputemodule.log"
CLIENT="10.0.2.2"
PORT=5000

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
if [ "$#" -eq 0 ]; then
    HOSTS=("aperturecomputemodule000" "aperturecomputemodule001")
else
    HOSTS=("$@")
fi

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
tmux new-session -d -s "$SESSION"

FIRST=1

for HOST in "${HOSTS[@]}"; do
    CMD=$(cat <<EOF
clear
echo "=== HOST: $HOST | CLIENT: $CLIENT | PORT: $PORT ==="
echo
ssh $USER@$HOST '
    echo "[remote] hostname: \$(hostname)"
    echo "[remote] process:"
    ps -ef | grep "[a]perturecomputemodule" || echo "not running"
    echo
    echo "[remote] log: $REMOTE_LOG"
    echo "----------------------------------------"
    tail -n 40 -f $REMOTE_LOG
'
EOF
)

    if [ "$FIRST" -eq 1 ]; then
        tmux send-keys -t "$SESSION:0.0" "$CMD" C-m
        FIRST=0
    else
        tmux split-window -t "$SESSION"
        tmux send-keys -t "$SESSION" "$CMD" C-m
        tmux select-layout -t "$SESSION" tiled
    fi

    CLIENT=$(increment_subnet "$CLIENT") || exit 1
done

tmux set-option -t "$SESSION" mouse on
tmux setw -t "$SESSION" synchronize-panes off
tmux attach -t "$SESSION"
