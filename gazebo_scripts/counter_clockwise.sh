if [ "$1" = "orca4" ]; then
   gz topic -t /model/$1/joint/thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10'
   gz topic -t /model/$1/joint/thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: 10'
   gz topic -t /model/$1/joint/thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: 10'
   gz topic -t /model/$1/joint/thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10'
   
else
   echo "Invalid model name"
   exit 1
fi
