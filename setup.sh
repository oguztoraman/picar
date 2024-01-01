#! /bin/bash
#
# SPDX-FileCopyrightText: Copyright (c) 2022-2024 Oğuz Toraman <oguz.toraman@tutanota.com>
# SPDX-License-Identifier: GPL-3.0-or-later

echo "Updating the system..."
sudo apt update
sudo apt upgrade -y
echo "Installing necessary softwares..."
sudo apt install -y g++ libopencv-dev cmake wiringpi
echo "Setup finished."
exit
