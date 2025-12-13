set(src_dirs        
    .
    app
    bsp
)

set(include_dirs  
    .
    app
    bsp
)

idf_component_register(
    SRC_DIRS ${src_dirs} 
    INCLUDE_DIRS ${include_dirs}
)