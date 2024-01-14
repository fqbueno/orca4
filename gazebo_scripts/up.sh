if [ "$1" = "orca4" ]; then
   gz topic -t /model/$1/joint/thruster5_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10'
   gz topic -t /model/$1/joint/thruster6_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10'
   gz topic -t /model/$1/joint/thruster7_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10'
   gz topic -t /model/$1/joint/thruster8_joint/cmd_thrust -m gz.msgs.Double -p 'data: -10'
   
else
   echo "Invalid model name"
   exit 1
fi
