# - LIB_CORE

aux_source_directory(./camera DIR_CAMERA)
set(DIR_CORE ${DIR_CORE} ${DIR_CAMERA})

aux_source_directory(./config_parse DIR_CONFIG_PARSE)
set(DIR_CORE ${DIR_CORE} ${DIR_CONFIG_PARSE})

aux_source_directory(./front_track DIR_FRONT_TRACK)
set(DIR_CORE ${DIR_CORE} ${DIR_FRONT_TRACK})

aux_source_directory(./odometry DIR_ODOMETRY)
set(DIR_CORE ${DIR_CORE} ${DIR_ODOMETRY})

aux_source_directory(./util DIR_UTIL)
set(DIR_CORE ${DIR_CORE} ${DIR_UTIL})

add_library(core ${DIR_CORE})