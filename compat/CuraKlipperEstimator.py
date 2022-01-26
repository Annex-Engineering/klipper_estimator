# Copyright (c) 2022 Lasse Dalegaard
# MIT licensed

from ..Script import Script
from tempfile import TemporaryDirectory
import subprocess
import shutil
import os

class KlipperEstimator(Script):
    """
    Runs klipper_estimator on the resulting gcode.
    """

    def getSettingDataString(self):
        return """{
            "name": "Klipper estimator",
            "key": "KlipperEstimator",
            "metadata": {},
            "version": 2,
            "settings":
            {
                "path":
                {
                    "label": "Path to klipper_estimator",
                    "description": "The path to the klipper_estimator binary.",
                    "type": "str",
                    "default_value": ""
                },
                "config_kind":
                {
                    "label": "Kind of config to use(file or moonraker_url)",
                    "description": "",
                    "type": "str",
                    "default_value": ""
                },
                "config_arg":
                {
                    "label": "Config argument",
                    "description": "Path for file, URL for Moonraker",
                    "type": "str",
                    "default_value": ""
                }
            }
        }"""

    def execute(self, data):
        with TemporaryDirectory() as work_dir:
            filename = os.path.join(work_dir, "work.gcode")
            with open(filename, 'w') as work_file:
                for line in data:
                    work_file.write(line + "\n")

            args = [
                self.getSettingValueByKey("path"),
                "--config_" + self.getSettingValueByKey("config_kind"),
                self.getSettingValueByKey("config_arg"),
                "post-process",
                filename,
            ]

            ret = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if ret.returncode != 0:
                raise RuntimeError("Failed to run klipper_estimator\n%s" % (ret.stdout,))

            with open(filename) as work_file:
                return work_file.readlines()
