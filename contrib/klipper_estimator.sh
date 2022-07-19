#!/usr/bin/env bash

#This is an example script that can be used for SLIC3R/PS/SS slicers or other
# if you figure out the exact name of the printer from the environment. 
#Uncommenting the following line might be useful to get a dump of all
#environment variables 
# set > ~/Downloads/set.env

#These variables can/should be customized for your liking and your setup
PRINTER=${SLIC3R_PHYSICAL_PRINTER_SETTINGS_ID}
KLIPPER_ESTIMATOR=~/.local/bin/klipper_estimator_osx
CONFIG_PATH=~/.local/share/klipper_estimator/
CONFIG_NAME=${PRINTER}.json
SUCCEED_ON_FAIL=true

#perhaps do not change anything below and open a support ISSUE on 
#klipper estimator github (https://github.com/Annex-Engineering/klipper_estimator)
set -x +e
if ping -c 1 ${PRINTER} ; then
  ${KLIPPER_ESTIMATOR} --config_moonraker_url http://"${PRINTER}" post-process "$@" 
  if [ $! && $DUMP_CONFIG ] ; then
    #we test the last command finished successfully so that there is a good
    # chance to get a valid config
    if [ -f $CONFIG_PATH/$CONFIG_NAME ]; then
      if cmp $CONFIG_PATH/$CONFIG_NAME <($KLIPPER_ESTIMATOR --config_moonraker_url  http://"${PRINTER}" dump-config ) ; then
        #dump config only if changed to prevent various FS bits from updating
        $KLIPPER_ESTIMATOR --config_moonraker_url  http://"${PRINTER}" dump-config > $CONFIG_PATH/$CONFIG_NAME
        echo "This is not an error. This is just let you know we have saved the backup".
        false
      fi
    else
      $KLIPPER_ESTIMATOR --config_moonraker_url  http://"${PRINTER}" dump-config > $CONFIG_PATH/$CONFIG_NAME
        echo "This is not an error. This is just let you know we have saved the backup".
      false
    fi
  fi
elif [ -f $CONFIG_PATH/$CONFIG_NAME ] ; then
  $KLIPPER_ESTIMATOR --config_file $CONFIG_PATH/$CONFIG_NAME  post-process "$@"
else
  echo "We couldn't postprocess the part but it's still OK"
  $SUCCEED_ON_FAIL
fi

