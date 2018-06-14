# add a launch file to the gps package
# if the file already exists, it will be overwritten by the default value
cd ~/como/workspace/src/nmea_navsat_driver
if [ ! -d ~/como/workspace/src/nmea_navsat_driver/launch ]; then
  mkdir launch
fi

cd launch
if [[ ! -e gps.launch ]]; then
  touch gps.launch
fi

tee gps.launch <<EOF
<launch>
    <node pkg="nmea_navsat_driver" type="nmea_serial_driver" name="gps">
        <param name="port" value="/dev/ttyACM0" />
        <param name="baud_rate" value="115200" />
    </node>
</launch>
EOF
