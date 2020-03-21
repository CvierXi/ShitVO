# - LIB_PC

aux_source_directory(./dataset_parse DIR_DATASET_PARSE)
set(DIR_PC ${DIR_PC} ${DIR_DATASET_PARSE})

aux_source_directory(./simulate DIR_SIMULATE)
set(DIR_PC ${DIR_PC} ${DIR_SIMULATE})

aux_source_directory(./visualize DIR_VISUALIZE)
set(DIR_PC ${DIR_PC} ${DIR_VISUALIZE})

add_library(pc ${DIR_PC})
target_link_libraries(pc core)
