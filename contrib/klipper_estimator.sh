# Copyright (c) 2022 Jan "Yenda" Trmal<jtrmal@gmail.com>
# MIT licensed

#!/usr/bin/env bash

#This is an example script that can be used for SLIC3R/PS/SS slicers or other
# if you figure out the exact name of the printer from the environment. 
#Uncommenting the following line might be useful to get a dump of all
#environment variables 
#set > ~/Downloads/set.env

#These variables can/should be customized for your liking and your setup
#On PS/SS the more appropriate value for PRINTER  would be probably 
#SLIC3R_PRINT_HOST but unfortunately, that one seems always empty. Might be 
#a bug or an intention. You can however name the physical printer settings 
#name the same as your printer hostname and ten this value would work. 
#Another option might be to set a custom variable in the PRINTER SETTINGS tab 
#to carry the domain name
#for example (in PS tab): 
#   PRINTER_DOMAIN_NAME=vsw.local
# and then the following code
#   eval ${SLIC3R_PRINTER_CUSTOM_VARIABLES}
#   PRINTER=$PRINTER_DOMAIN_NAME
#In that case do not forget to remove the following definition of the PRINTER 
#variable. Using eval in the code has certain security implications
#so do it at your own risk

PRINTER=${SLIC3R_PHYSICAL_PRINTER_SETTINGS_ID}
KLIPPER_ESTIMATOR=~/.local/bin/klipper_estimator_osx
CONFIG_PATH=~/.local/share/klipper_estimator/
CONFIG_NAME=${PRINTER}.json
SUCCEED_ON_FAIL=true   #do you want the export of gcode to always succeed?

#perhaps do not change anything below and open a support ISSUE on 
#klipper estimator github (https://github.com/Annex-Engineering/klipper_estimator)
set -x
if ping -c 1 ${PRINTER} ; then
  ${KLIPPER_ESTIMATOR} --config_moonraker_url http://"${PRINTER}" post-process "$@" 
  if [ $? -eq 0 ]  && $DUMP_CONFIG  ; then
    #we tested the last command finished successfully so that there is a good
    # chance to get a valid config
    tmpfile=`mktemp`
    $KLIPPER_ESTIMATOR --config_moonraker_url  http://"${PRINTER}" dump-config > $tmpfile
    if [ -s $tmpfile ] ; then
      if [ -f $CONFIG_PATH/$CONFIG_NAME ] ; then
        if ! cmp $CONFIG_PATH/$CONFIG_NAME $tmpfile ; then
          cp $tmpfile $CONFIG_PATH/$CONFIG_NAME
          echo "This is not an error. This is just let you know we have saved the backup. Just re-run the export."
          false
        fi
      else
        $KLIPPER_ESTIMATOR --config_moonraker_url  http://"${PRINTER}" dump-config > $CONFIG_PATH/$CONFIG_NAME
        echo "This is not an error. This is just let you know we have saved the backup. Just re-run the export."
        false
      fi
    fi
    rm $tmpfile
  fi
elif [ -f $CONFIG_PATH/$CONFIG_NAME ] ; then
  $KLIPPER_ESTIMATOR --config_file $CONFIG_PATH/$CONFIG_NAME  post-process "$@"
else
  echo "We couldn't postprocess the part but it's still OK"
  $SUCCEED_ON_FAIL
fi

