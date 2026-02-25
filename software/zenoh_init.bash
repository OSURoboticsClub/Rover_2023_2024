export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='connect/endpoints={ router: ["tcp/10.42.0.100:7447"], peer: ["tcp/127.0.0.1:7447"]}'
alias kill_zenoh="pkill rmw_zenohd; while pgrep rmw_zenohd; do sleep 0.1; done; echo 'Done.'"

pgrep rmw_zenohd > /dev/null
result=$?

if [ $result -eq 1 ]; then
        echo "Zenoh router not running. Terminal Configured for Zenoh."
        #start_zenoh
elif [ $result -eq 0 ]; then
        echo "Terminal Configured for Zenoh. Zenoh router already running."
else
        echo "Failed to verify status of Zenoh router"
fi

ros2 daemon stop > /dev/null
