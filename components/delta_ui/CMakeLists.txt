# GUI Guider 生成的资源（generated/、images/、fonts/）由工具整体重新生成，
# 因此这里用通配方式统一收集，不必逐个列出。
file(GLOB_RECURSE GUI_GUIDER_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/generated/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/images/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/fonts/*.c
)

idf_component_register(
        SRCS ${GUI_GUIDER_SOURCES}
        INCLUDE_DIRS
        "include"     # 对外公开头文件
        "generated"   # GUI Guider 生成的头文件
        "images"
        "fonts"
        "."
        REQUIRES lvgl delta move end_effector
)
