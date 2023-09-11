#!/bin/bash
python_path=$1
echo python_path
cd $python_path
package_list=("python3-rpi.gpio" "python3-dev" "python3-smbus" "python3-pip")
for check_package in ${package_list[@]}; do
  PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $check_package|grep "install ok installed")
  echo Checking for $check_package: $PKG_OK
  if [ "" = "$PKG_OK" ]; then
    echo "No $check_package. Setting up $check_package."
    ./setup.sh
    break
  fi
done

echo "Verifying pip list necessaries.."

while read package2; do
  if ! pip show $package2 > /dev/null 2>&1; then
    ./setup.sh
  else
    echo "$package2 is already installed"
  fi

done < requirements.txt
#for check_package in ${package_list2[@]}; do
#  echo $check_package
#  pip3 show $check_package &> /dev/null
#  if [ $? -eq 0 ]; then
#      echo "$check_package is already installed!"
#  else
#      echo "$check_package IS NOT FOUND! Installing.."
#      ./setup.sh
#      break
#  fi
#done
#location="$(pip3 show feagi_agent | grep Location: )"
#echo $location
#new_location="${location/Location: /""}"
#echo $new_location
#python3 $new_location/feagi_agent/robot_controllers/freenove_smartcar/freenove_SmartCar.py

