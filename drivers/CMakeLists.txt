# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_FUEL_GAUGE fuel_gauge)

add_subdirectory_ifdef(CONFIG_CHARGER charger)

add_subdirectory_ifdef(CONFIG_SENSOR sensor)