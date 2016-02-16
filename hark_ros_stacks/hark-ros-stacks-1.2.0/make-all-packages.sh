#!/bin/sh

#echo $(ls -d */)

./clean-all-packages.sh

for package in $(ls -d */)
do
  
  if [ ${package} != "build/" ] && [ ${package} != "bin/" ] && [ ${package} != "lib/" ]
  then
    
    echo "Making package: ${package}"  
    cd ${package}
    rosmake
    cd ..
  
  fi
  
done

