FILE(GLOB U8G2_SRCS u8g2/*.c)

idf_component_register(
    SRCS 
    ${U8G2_SRCS}       
    INCLUDE_DIRS "u8g2" "src"
)