#!/bin/bash

#sudo apt-get install xterm	
name="FEAGI FOXY ARDIUNO TOPIC" #easier to remember which to run on.
echo $name
# opens terminal but then I can't control terminal afterwards

if [[ "$name" == 'FEAGI' ]];then
	echo $name && xterm -hold -e "'echo Hello My World'; cd; pwd'" &
elif  [ $name == 'FOXY' ]
then
	xterm -hold -e "echo test"
else
	echo "else"
fi
done
