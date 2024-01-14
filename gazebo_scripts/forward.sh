

if ["$1" = "orca4"]; then
    gz topic -t /model/$1/thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10' 
    gz topic -t /model/$1/thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10' 
    gz topic -t /model/$1/thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: 10' 
    gz topic -t /model/$1/thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: 10' 
else
    echo "Invalid model name"
    exit 1
fi
