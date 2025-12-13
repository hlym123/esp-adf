set(src_dirs        
    .
    bsp
)

set(include_dirs  
    .
    bsp
)

idf_component_register(
    SRC_DIRS ${src_dirs} 
    INCLUDE_DIRS ${include_dirs}
)