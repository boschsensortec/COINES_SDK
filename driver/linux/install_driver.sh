#!/bin/sh -x
sudo cp bst_coines.rules /etc/udev/rules.d/bst_coines.rules
sudo udevadm control --reload-rules

